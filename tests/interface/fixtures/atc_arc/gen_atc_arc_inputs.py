#!/usr/bin/env python3
"""pe inputs for the 90 deg ATC arc (M273): boundary surface, particles, slab planes, deck.

Usage: gen_atc_arc_inputs.py --out DIR [--particles N] [--radius R] [--seed S]
                             [--planes FILE | --slabs N] [--nseg N] [--dphi DEG]
                             [--guard CM] [--dm-resolution N] [--steps N]

Writes into DIR:

  atc_arc_boundary.obj            closed surface of the fluid domain, outward normals
  particles.xyz                   start positions of the spheres, one "x y z" per line
  sorted_vertices_by_x_world.txt  centreline of the arc (the ATC setups read it)
  single_plane.txt                slab planes, "px py pz nx ny nz" per line: a copy of FILE
                                  (--planes, the file the CFD mesh was partitioned with), or
                                  N slabs cut on layer faces of the hex mesh (--slabs, default 4)
  example.json                    deck read by setupATC / setupATCSerial

The files next to this script are the fixture of pe_atc_arc_containment
(--out . --particles 400 --nseg 64 --dphi 1 --steps 600).

Geometry [cm]: the liquid segment of the Archimedes tube crystalliser as meshed for the CFD
run (mesh M273).  Coil axis = z, centreline radius R_C, tube radius R_T, arc from PHI_A (z = 0)
to PHI_B.  Every section of the tube with a plane through the coil axis is a circle of radius
R_T around the centreline point, as in the hex mesh.  Both ends are closed by the part of a
sphere of radius R_SPH that lies beyond the end plane; the sphere passes through the rim
circle of the end section (the surfaces the CFD solver projects its boundary nodes onto).

The deck holds the centre of mass of the surface as "position": pe moves a mesh read from a
file so that its centre of mass sits at that position, so this is the value that leaves the
surface where the file puts it.

Bounding-box guard (--guard CM, off by default): two tiny closed tetrahedra at the corners of
the bounding box grown by the margin, outside the fluid domain.  They widen the bounding box
of the mesh.  pe needed that before a mesh with an inverted DistanceMap took the box of the
map's grid as its own (TriangleMesh::widenBoundingBoxForDistanceMap()): contacts were only
generated inside the box of the vertices, and where the wall touches that box (the bottom of
the arc) spheres sank through it.  Kept to reproduce that with an older pe.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np


R_C = 2.5            # coil centreline radius
R_T = 0.25           # tube inner radius
PITCH = 0.7          # axial advance per turn
PHI_A = -45.0        # arc start (z = 0), the +x end
PHI_B = -135.0       # arc end, the -x end
R_SPH = 2.0 * R_T    # cap sphere radius

SGN = 1.0 if PHI_B > PHI_A else -1.0          # direction of travel from A to B in phi
PSI_MAX = math.asin(R_T / R_SPH)              # half opening angle of the caps
BACK = math.sqrt(R_SPH ** 2 - R_T ** 2)       # set-back of the sphere centres


def z_of(phi_deg):
    return -PITCH * (phi_deg - PHI_A) / 360.0


def centre(phi_deg):
    p = math.radians(phi_deg)
    return np.array([R_C * math.cos(p), R_C * math.sin(p), z_of(phi_deg)])


def e_rad(phi_deg):
    p = math.radians(phi_deg)
    return np.array([math.cos(p), math.sin(p), 0.0])


def e_phi(phi_deg):
    p = math.radians(phi_deg)
    return np.array([-math.sin(p), math.cos(p), 0.0])


E_Z = np.array([0.0, 0.0, 1.0])
N_OUT_A = -SGN * e_phi(PHI_A)                 # outward normal of the end plane at A
N_OUT_B = +SGN * e_phi(PHI_B)
C_SPH_A = centre(PHI_A) - BACK * N_OUT_A      # cap sphere centres
C_SPH_B = centre(PHI_B) - BACK * N_OUT_B


# --------------------------------------------------------------------------------------------
# analytic clearance (distance of a point to the boundary, negative outside)
# --------------------------------------------------------------------------------------------
def clearance(p):
    """Vectorised over an (n, 3) array.  Same definition as in the pe containment test."""
    p = np.atleast_2d(p)
    phi = np.degrees(np.arctan2(p[:, 1], p[:, 0]))
    lo, hi = min(PHI_A, PHI_B), max(PHI_A, PHI_B)
    rho = np.hypot(p[:, 0], p[:, 1])
    zc = -PITCH * (phi - PHI_A) / 360.0
    wall = R_T - np.hypot(rho - R_C, p[:, 2] - zc)
    out = np.where((phi >= lo) & (phi <= hi), wall, -np.inf)
    for c, n in ((C_SPH_A, N_OUT_A), (C_SPH_B, N_OUT_B)):
        d = p - c
        dist = np.linalg.norm(d, axis=1)
        along = d @ n
        beyond = along > BACK                               # beyond the end plane
        in_cone = along >= dist * math.cos(PSI_MAX)         # nearest sphere point is on the cap
        cap = R_SPH - dist
        out = np.where(beyond, cap, np.where(in_cone, np.minimum(out, cap), out))
    return out


# --------------------------------------------------------------------------------------------
# closed boundary surface
# --------------------------------------------------------------------------------------------
def boundary_surface(nseg, dphi, ncap):
    nring = int(round(abs(PHI_B - PHI_A) / dphi)) + 1
    phis = np.linspace(PHI_A, PHI_B, nring)
    th = np.linspace(0.0, 2.0 * math.pi, nseg, endpoint=False)
    verts = []
    for ph in phis:                                           # wall rings
        c, er = centre(ph), e_rad(ph)
        verts.append(c + R_T * (np.cos(th)[:, None] * er + np.sin(th)[:, None] * E_Z))
    faces = []
    for i in range(nring - 1):
        for j in range(nseg):
            j1 = (j + 1) % nseg
            a, b, c, d = i * nseg + j, i * nseg + j1, (i + 1) * nseg + j1, (i + 1) * nseg + j
            faces += [(a, b, c), (a, c, d)]
    nv = nring * nseg
    for ring, ph, csph, nout in ((0, PHI_A, C_SPH_A, N_OUT_A),
                                 (nring - 1, PHI_B, C_SPH_B, N_OUT_B)):
        er = e_rad(ph)
        prev = [ring * nseg + j for j in range(nseg)]
        for k in range(ncap - 1, 0, -1):                      # cap rings, rim -> pole
            psi = PSI_MAX * k / ncap
            verts.append(csph + R_SPH * (math.cos(psi) * nout + math.sin(psi) *
                                         (np.cos(th)[:, None] * er + np.sin(th)[:, None] * E_Z)))
            cur = [nv + j for j in range(nseg)]
            nv += nseg
            for j in range(nseg):
                j1 = (j + 1) % nseg
                faces += [(prev[j], prev[j1], cur[j1]), (prev[j], cur[j1], cur[j])]
            prev = cur
        verts.append((csph + R_SPH * nout)[None])
        faces += [(prev[j], prev[(j + 1) % nseg], nv) for j in range(nseg)]
        nv += 1
    verts = np.concatenate(verts)
    faces = np.array(faces)
    # outward orientation, decided per connected part by the sign of the total volume: the wall
    # and both caps are built with one winding each, so flip the parts that point inward
    fc = verts[faces].mean(axis=1)
    nrm = np.cross(verts[faces[:, 1]] - verts[faces[:, 0]], verts[faces[:, 2]] - verts[faces[:, 0]])
    eps = 1e-4
    inward = clearance(fc + eps * nrm / np.linalg.norm(nrm, axis=1)[:, None]) > \
        clearance(fc - eps * nrm / np.linalg.norm(nrm, axis=1)[:, None])
    faces[inward] = faces[inward][:, ::-1]
    return verts, faces


def add_bbox_guard(verts, faces, margin, size=1.0e-3):
    """Two tiny outward-oriented tetrahedra at the corners of the bounding box grown by margin."""
    lo, hi = verts.min(0) - margin, verts.max(0) + margin
    extra_v, extra_f, n = [], [], len(verts)
    for corner, sign in ((lo, 1.0), (hi, -1.0)):
        t = np.array([corner, corner + [sign * size, 0, 0], corner + [0, sign * size, 0],
                      corner + [0, 0, sign * size]])
        f = np.array([(0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3)])
        if np.dot(t[1] - t[0], np.cross(t[2] - t[0], t[3] - t[0])) < 0.0:
            f = f[:, ::-1]
        extra_v.append(t)
        extra_f.append(f + n)
        n += 4
    return np.concatenate([verts] + extra_v), np.concatenate([faces] + extra_f)


def volume_and_com(verts, faces):
    a, b, c = verts[faces[:, 0]], verts[faces[:, 1]], verts[faces[:, 2]]
    v6 = np.einsum("ij,ij->i", a, np.cross(b, c))
    vol = v6.sum() / 6.0
    com = ((a + b + c) * v6[:, None]).sum(axis=0) / (4.0 * v6.sum())
    return vol, com


def check_closed(faces):
    e = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    key = e[:, 0].astype(np.int64) * (faces.max() + 1) + e[:, 1]
    rev = e[:, 1].astype(np.int64) * (faces.max() + 1) + e[:, 0]
    return len(np.unique(key)) == len(key) and set(key.tolist()) == set(rev.tolist())


def write_obj(path, verts, faces):
    with open(path, "w") as f:
        f.write("# Fluid domain of the 90 deg ATC arc (M273), cm, outward normals\n")
        for v in verts:
            f.write("v %.12g %.12g %.12g\n" % tuple(v))
        for t in faces + 1:
            f.write("f %d %d %d\n" % tuple(t))


# --------------------------------------------------------------------------------------------
# particles
# --------------------------------------------------------------------------------------------
def particle_positions(n, radius, seed):
    """n points of a cubic lattice (pitch 3 radii) at least 2.5 radii away from the boundary."""
    pitch = 3.0 * radius
    lo = np.array([-R_C - R_T, -R_C - R_T, -R_T])
    hi = np.array([R_C + R_T, 0.0, z_of(PHI_B) + R_T])
    axes = [np.arange(l + 0.5 * pitch, h, pitch) for l, h in zip(lo, hi)]
    keep = []
    for x in axes[0]:                                   # slice-wise to bound the memory
        yy, zz = np.meshgrid(axes[1], axes[2], indexing="ij")
        pts = np.stack([np.full(yy.size, x), yy.ravel(), zz.ravel()], -1)
        keep.append(pts[clearance(pts) >= 2.5 * radius])
    pts = np.concatenate(keep)
    if n > len(pts):
        raise SystemExit("only %d lattice sites for %d particles" % (len(pts), n))
    rng = np.random.default_rng(seed)
    return pts[np.sort(rng.choice(len(pts), size=n, replace=False))], len(pts)


# --------------------------------------------------------------------------------------------
def slab_planes(n_slabs, layer_deg):
    """Planes through the coil axis on layer faces of the hex mesh (as gen_case.py)."""
    n_lay = int(round(abs(PHI_B - PHI_A) / layer_deg))
    base, extra = divmod(n_lay, n_slabs)
    counts = [base] * n_slabs
    for k in range(extra):
        counts[(k // 2) if k % 2 == 0 else (n_slabs - 1 - k // 2)] += 1
    cuts = [PHI_A + SGN * layer_deg * sum(counts[:k + 1]) for k in range(n_slabs - 1)]
    planes = [((0.0, 0.0, 0.0), tuple(-SGN * e_phi(c))) for c in cuts]
    planes.append(((0.0, 0.0, 0.0), tuple(e_rad(0.5 * (PHI_A + PHI_B)))))
    return planes, cuts, counts


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", required=True)
    ap.add_argument("--particles", type=int, default=2000)
    ap.add_argument("--radius", type=float, default=0.0092)
    ap.add_argument("--seed", type=int, default=98765)
    ap.add_argument("--planes", default=None,
                    help="copy this plane file (the CFD partition) instead of generating slabs")
    ap.add_argument("--slabs", type=int, default=4)
    ap.add_argument("--steps", type=int, default=2000, help="timesteps_ of the deck")
    ap.add_argument("--layers", type=int, default=66, help="element layers of the hex mesh")
    ap.add_argument("--nseg", type=int, default=96, help="segments around the tube")
    ap.add_argument("--dphi", type=float, default=0.5, help="ring spacing along the arc [deg]")
    ap.add_argument("--ncap", type=int, default=10, help="rings per cap")
    ap.add_argument("--guard", type=float, default=0.0,
                    help="margin of a bounding-box guard [cm], only for an older pe (see the header)")
    ap.add_argument("--dm-resolution", type=int, default=200)
    ap.add_argument("--dm-tolerance", type=int, default=6)
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    verts, faces = boundary_surface(args.nseg, args.dphi, args.ncap)
    domain_lo, domain_hi = verts.min(0), verts.max(0)
    if args.guard > 0.0:
        verts, faces = add_bbox_guard(verts, faces, args.guard)
    assert check_closed(faces), "boundary surface is not closed / consistently oriented"
    vol, com = volume_and_com(verts, faces)
    h = R_SPH - BACK
    vol_ref = math.pi * R_T ** 2 * R_C * math.radians(abs(PHI_B - PHI_A)) \
        + 2.0 * math.pi * h * (3.0 * R_T ** 2 + h ** 2) / 6.0
    assert vol > 0.0, "boundary surface points inward"
    write_obj(out / "atc_arc_boundary.obj", verts, faces)
    print("atc_arc_boundary.obj: %d vertices, %d triangles, closed; volume %.6f cm^3 "
          "(analytic %.6f, %+.3f %%); centre of mass (%.9f, %.9f, %.9f)"
          % (len(verts), len(faces), vol, vol_ref, 100.0 * (vol / vol_ref - 1.0), *com))
    print("fluid domain: %s .. %s%s"
          % (np.round(domain_lo, 5), np.round(domain_hi, 5),
             "; bounding-box guard %g cm" % args.guard if args.guard > 0.0 else ""))

    pts, sites = particle_positions(args.particles, args.radius, args.seed)
    np.savetxt(out / "particles.xyz", pts, fmt="%.12g")
    print("particles.xyz: %d of %d lattice sites, radius %g, min clearance %.4f"
          % (len(pts), sites, args.radius, clearance(pts).min()))

    cl = np.array([centre(p) for p in np.linspace(PHI_A, PHI_B, 91)])
    np.savetxt(out / "sorted_vertices_by_x_world.txt", cl[::-1], fmt="%.12g")

    if args.planes:
        # the file the CFD mesh was partitioned with: both sides must read the same planes
        text = Path(args.planes).read_text()
        (out / "single_plane.txt").write_text(text)
        rows = np.array([l.split() for l in text.split("\n") if l.strip()], float)
        n_slabs = len(rows)
        cuts = [float(np.degrees(np.arctan2(n[1], n[0]))) + 90.0 * SGN for n in rows[:-1, 3:]]
        source = "copied from %s" % args.planes
    else:
        n_slabs = args.slabs
        planes, cuts, counts = slab_planes(n_slabs, abs(PHI_B - PHI_A) / args.layers)
        with open(out / "single_plane.txt", "w") as fh:
            for pt, nrm in planes:
                fh.write("%.15g %.15g %.15g %.15g %.15g %.15g\n" % (pt + nrm))
        source = "generated, %s layers per slab" % counts
    phi_p = np.degrees(np.arctan2(pts[:, 1], pts[:, 0]))
    edges = [PHI_A - 90.0 * SGN] + cuts + [PHI_B + 90.0 * SGN]      # the end slabs hold the caps
    per_slab = [int(((phi_p <= max(a, b)) & (phi_p > min(a, b))).sum())
                for a, b in zip(edges[:-1], edges[1:])]
    print("single_plane.txt: %d slabs (%s), cuts at %s deg; particles per slab %s"
          % (n_slabs, source, ", ".join("%.4f" % c for c in cuts), per_slab))

    deck = {
        "timesteps_": args.steps,
        "stepsize_": 0.0005,
        "substeps_": 1,
        "processesX_": n_slabs,
        "processesY_": 1,
        "processesZ_": 1,
        "vtk_": False,
        "visspacing_": 50,
        "useCheckpointer_": False,
        "resume_": False,
        "checkpoint_path_": "checkpoints",
        "benchRadius_": args.radius,
        "particleDensity_": 1.42,
        "fluidViscosity_": 0.058,
        "fluidDensity_": 1.0,
        "gravity_": [0.0, -980.0, 0.0],
        "packingMethod_": "external",
        "xyzFilePath_": "particles.xyz",
        "domainBoundary_": {
            "enabled": True,
            "file": "atc_arc_boundary.obj",
            "position": [float("%.12g" % c) for c in com],
            "fixed": True,
            "visible": True,
            "distanceMap": {
                "enabled": True,
                "resolution": args.dm_resolution,
                "tolerance": args.dm_tolerance,
                "invertForDomainBoundary": True,
                "writeVti": False,
            },
        },
    }
    (out / "example.json").write_text(json.dumps(deck, indent=2) + "\n")
    span = (verts.max(0) - verts.min(0)).max()
    print("example.json: DistanceMap resolution %d -> spacing %.5f cm (%.2f particle radii), "
          "padding %d cells" % (args.dm_resolution, span / args.dm_resolution,
                                span / args.dm_resolution / args.radius, args.dm_tolerance))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
