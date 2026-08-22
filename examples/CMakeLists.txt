# Serial Examples
add_subdirectory(body_removal)  # Example showing body removal during simulation
add_subdirectory(box_domain)
add_subdirectory(cylinder)
add_subdirectory(decomposition)
add_subdirectory(detection)
add_subdirectory(kdop_example)  # Example showing k-DOP extraction from TriMeshDopBoundary
add_subdirectory(math_tests)
add_subdirectory(rotation_collision)  # Example showing translation-fixed vs fully-fixed rigid bodies
add_subdirectory(simple_granular)
add_subdirectory(triangle_mesh)
add_subdirectory(usage_demo)
add_subdirectory(simple_demo)
add_subdirectory(basic_lubrication)
add_subdirectory(lubrication_demo)
add_subdirectory(capsule_spin)
add_subdirectory(vtk_shapes_ascii)
add_subdirectory(vtk_writer_test)  # VTK Writer verification test
add_subdirectory(sphere_column_serial)  # Serial sphere column with ShortRangeRepulsion solver
add_subdirectory(sphere_column_serial_hcl)  # Serial sphere column with HardContactAndFluid solver + lubrication stage
add_subdirectory(shear_cell_kroupa)  # Level-2 rheology skeleton (Kroupa 2016 lubrication)

# CGAL Examples
if( PE_USE_CGAL )
    add_subdirectory(cgal_examples)  # Example showing CGAL oriented bounding box functionality
    add_subdirectory(convex_hull)  # Example showing CGAL convex hull computation for triangle meshes
    add_subdirectory(atc)  # ATC (Annular Tube Configuration) standalone example with timing
    add_subdirectory(mesh_spin)  # Example showing mesh rotation with DistanceMap collision detection
endif( PE_USE_CGAL )

# MPI Examples
if( PE_USE_MPI )
    add_subdirectory(decomposition_geometry)
    add_subdirectory(mpi3periodic)
    add_subdirectory(mpibench)
    add_subdirectory(mpiboxes)
    add_subdirectory(mpicapsules)
    add_subdirectory(mpidem)
    add_subdirectory(mpidomain)
    add_subdirectory(mpiperiodic)
    # Retired: mpiperiodic_dev and mpiplanedecomposition. Both were scratch variants of the old
    # mpiperiodic and are not built any more. They read plane geometry from files that do not
    # exist in the repo (planes.txt / my_planes.txt), then index the resulting empty vector; they
    # declare `int center[2]` for a 2D Cartesian topology and then read center[2] out of bounds;
    # and mpiperiodic itself was not even periodic (periods = { false, false }). mpiperiodic has
    # been rewritten from scratch as a self-verifying periodic transport test. The sources are
    # kept in the tree for reference only.
    # add_subdirectory(mpiperiodic_dev)
    # add_subdirectory(mpiplanedecomposition)
    add_subdirectory(mpispheres)
    add_subdirectory(mpiunion)
    add_subdirectory(rectigrid_demo)
    add_subdirectory(trimeshdop_demo) # TriMeshDopBoundary example using OBJ files for domain decomposition
endif( PE_USE_MPI )
