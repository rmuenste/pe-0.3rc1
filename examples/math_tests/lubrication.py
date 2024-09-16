import numpy as np
import matplotlib.pyplot as plt

# Constants
dp = np.array([12.7e-3, 12.7e-3])   # diameter, m
rhop = np.array([1400, 1400])       # density, kg/m^3
Y = np.array([2.8e9, 2.8e9])        # Young's modulus, Pa
nu = np.array([0.35, 0.35])         # Poisson's ratio, -
edry = 0.97                         # dry restitution coefficient, -
sigma = np.array([796e-9, 796e-9])  # surface roughness, m
mufDry = 0.2                        # dry friction coefficient, -
mufLub = 0.1                        # lubricated friction coefficient, -

# Fluid properties
rhof = 1e3                          # density, kg/m^3
etaf = 1e-3                         # dynamic viscosity, Pa s
lubrication = True                  # enable lubrication model

# Initial conditions
St = 50                             # total Stokes number, -
theta0 = np.deg2rad(20)             # angle of impact, rad

# Derived properties
np_ = 2                             # number of particles, -
dp_ = np.prod(dp) / np.sum(dp)      # reduced diameter, m
rp = dp / 2                         # radius, m
rp_ = dp_ / 2                       # reduced radius, m
vp = np.pi / 6 * dp**3              # volume, m^3
Mp = vp * rhop                      # mass, kg
Mp_ = np.prod(Mp) / np.sum(Mp)      # reduced mass, kg

# Reduced shear modulus and Young's modulus
G_ = 1 / (2 * (2 - nu[0]) * (1 + nu[0]) / Y[0] + 2 * (2 - nu[1]) * (1 + nu[1]) / Y[1])
Y_ = 1 / ((1 - nu[0]**2) / Y[0] + (1 - nu[1]**2) / Y[1])
beta = np.log(edry) / np.sqrt(np.log(edry)**2 + np.pi**2)

# Initial conditions
vr0 = St * 6 * np.pi * etaf * rp_**2 / Mp_  # initial speed, m/s
h0 = dp_                                    # initial gap height, m

phi0 = np.array([0, 0])                     # initial orientation, rad
omega0 = np.array([0, 0])                   # initial rotational velocity, rad/s

x0 = np.array([[-np.cos(theta0) * h0 - rp[0], rp[1]],
               [-np.sin(theta0) * h0, 0]])  # initial position, m
v0 = np.array([[vr0 * np.cos(theta0), 0],
               [vr0 * np.sin(theta0), 0]])  # initial velocity, m/s

vrn0 = vr0 * np.cos(theta0)  # initial normal speed, m/s
vrt0 = vr0 * np.sin(theta0)  # initial tangential speed, m/s

# Lubrication properties
hco = rp_                    # cut-off distance, m
Stn = St * np.cos(theta0)    # normal Stokes number, -
Stt = St * np.sin(theta0)    # tangential Stokes number, -

# Solver settings
tc = 2.87 * (Mp_**2 / (rp_ * Y_**2 * vr0))**(1/5)  # contact time, s
dt = tc / 50                                       # time step, s
OoM = 10**np.floor(np.log10(dt))                   # order of magnitude of dt
dt = np.round(dt / OoM) * OoM                      # round dt to 1 significant digit
tend = 2 * h0 / vr0                                # end time, s
nt = int(np.ceil((tend + 1e-12) / dt))             # number of time steps, -

t = np.arange(nt) * dt                             # time values, s
plotEvery = nt // 200                              # plot interval, -

# Minimum approach
hmins = np.mean(sigma)  # minimum approach distance due to roughness, m

# Minimum approach distance due to deformation
hmine = 0.37 * ((etaf * vrn0 / Y_)**2 * rp_**3)**(1/5)
hmin = max(hmine, hmins)  # minimum approach distance, m

# Effective friction coefficient
if hmins > hmine or not lubrication:
    mufEff = mufDry
else:
    mufEff = mufLub

# Preallocate arrays
F = np.zeros((2, np_, nt))        # total force, N
Fcn = np.zeros(nt)                # normal contact force magnitude, N
Fct = np.zeros(nt)                # tangential contact force magnitude, N
Fln = np.zeros(nt)                # normal lubrication force magnitude, N
Fn = np.zeros(nt)                 # total normal force magnitude, N

v = np.zeros((2, np_, nt))        # velocity, m/s
v[:, :, 0] = v0

x = np.zeros((2, np_, nt))        # position, m
x[:, :, 0] = x0

phi = np.zeros((np_, nt))         # orientation, rad
phi[:, 0] = phi0

omega = np.zeros((np_, nt))       # rotational velocity, rad/s
omega[:, 0] = omega0

tau = np.zeros((np_, nt))         # torque, Nm

Epot = np.zeros(nt)               # potential energy, J
deltat = np.zeros(nt)             # tangential overlap, m

# Main loop
for i in range(1, nt):
    # Velocity Verlet integration, position
    x[:, :, i] = x[:, :, i - 1] + v[:, :, i - 1] * dt + F[:, :, i - 1] / Mp[:, np.newaxis] * dt**2 / 2
    phi[:, i] = phi[:, i - 1] + omega[:, i - 1] * dt + tau[:, i - 1] / (2 / 5 * Mp * rp**2) * dt**2 / 2

    rij = x[:, 1, i] - x[:, 0, i]        # distance vector, m
    nij = rij / np.linalg.norm(rij)      # normal unit vector, -
    tij = np.array([-nij[1], nij[0]])    # tangential unit vector, -
    h = np.linalg.norm(rij) - np.sum(rp) # gap height, m

    vr = v[:, 0, i - 1] - v[:, 1, i - 1] # relative velocity, m/s
    vrn = np.dot(nij, vr)                # normal relative velocity, m/s
    vrt = np.dot(tij, vr) + omega[0, i - 1] * rp[0] + omega[1, i - 1] * rp[1]  # tangential relative velocity, m/s

    # Overlap
    if lubrication:
        deltan = max(0, hmin - h)    # overlap, m
    else:
        deltan = max(0, -h)        

    # Contact force
    if deltan > 0:
        # Normal contact force
        sn = 2 * Y_ * np.sqrt(rp_ * deltan)                 # Pa m = N/m
        kn = 4 / 3 * Y_ * np.sqrt(rp_ * deltan)             # Pa m = N/m
        gamman = -2 * np.sqrt(5 / 6) * beta * np.sqrt(sn * Mp_)    # sqrt(Pa m kg) = kg/s
        Fcn[i] = -(kn * deltan + gamman * vrn)              # N/m*m = kg/s*m/s = N

        Epot[i] = 1 / 2 * kn * deltan**2
    else:
        Fcn[i] = 0

    if h < 0:
        # Tangential contact force
        deltat[i] = deltat[i - 1] + vrt * dt
        st = 8 * G_ * np.sqrt(rp_ * deltan)                 # Pa m = N/m
        kt = 8 * G_ * np.sqrt(rp_ * deltan)                 # Pa m = N/m
        gammat = -2 * np.sqrt(5 / 6) * beta * np.sqrt(st * Mp_)    # sqrt(Pa m kg) = kg/s
        Fct[i] = -kt * deltat[i]
        if abs(Fct[i]) > mufEff * abs(Fcn[i]):
            Fct[i] = mufEff * abs(Fcn[i]) * np.sign(Fct[i])
            deltat[i] = abs(Fct[i]) / kt * np.sign(deltat[i])
        else:
            Fct[i] = Fct[i] - gammat * vrt

        Epot[i] = Epot[i] + 1 / 2 * kt * deltat[i]**2
    else:
        Fct[i] = 0

    # Lubrication force
    if lubrication:
        hlub = max(h, hmin)
        Fln[i] = -6 * np.pi * etaf * vrn * rp_**2 / hlub  # lubrication force, N

    # Total normal force
    if lubrication:
        if h > hco:
            Fn[i] = 0
        elif h <= hco and h > hmin:
            Fn[i] = Fln[i]
        elif h <= hmin and h > 0:
            Fn[i] = (1 - h / hmin) * Fcn[i] + h / hmin * Fln[i]
        elif h <= 0:
            Fn[i] = Fcn[i]
    else:
        Fn[i] = Fcn[i]

    # Apply force and torque
    F[:, 0, i] = Fn[i] * nij + Fct[i] * tij
    F[:, 1, i] = -Fn[i] * nij - Fct[i] * tij
    tau[:, i] = Fct[i] * rp

    # Velocity Verlet integration, velocity
    v[:, :, i] = v[:, :, i - 1] + (F[:, :, i - 1] + F[:, :, i]) / 2 / Mp[:, np.newaxis] * dt
    omega[:, i] = omega[:, i - 1] + (tau[:, i - 1] + tau[:, i]) / 2 / (2 / 5 * Mp * rp**2) * dt

    # Plotting
    if i % plotEvery == 0 or i == nt - 1:
        plt.clf()
        plt.plot([x0[0, 0] - 1.5 * rp[0], x0[0, 1] + 1.5 * rp[1]], [0, 0], color='gray')  # x axis
        plt.plot([0, 0], np.array([-1, 1]) * (np.abs(x0[1, 0]) + 1.5 * max(rp)), color='gray')        # y axis
        for j in range(2):
            circle = plt.Circle((x[0, j, i], x[1, j, i]), rp[j], edgecolor='black', facecolor='none')
            plt.gca().add_patch(circle)
            plt.plot(x[0, j, i], x[1, j, i], 'k+')
            plt.plot(x[0, j, :i + 1], x[1, j, :i + 1], 'k-')
            for k in range(4):
                angle = phi[j, i] + k * np.pi / 2
                plt.plot(x[0, j, i] + np.cos(angle) * rp[j] / 2, x[1, j, i] + np.sin(angle) * rp[j] / 2, 'k.')
        plt.text(0.95 * x0[0, 0] - 1.5 * rp[0], -0.9 * (abs(x0[1, 0]) + 1.5 * max(rp)),
                 f't* = {t[i] / (rp[0] / vr0):.2f}', fontsize=11)
        plt.axis('equal')
        plt.xlim([x0[0, 0] - 1.5 * rp[0], x0[0, 1] + 1.5 * rp[1]])
        #plt.ylim([-1 * (abs(x0[1, 0]) + 1.5 * max(rp)), 1 * (abs(x0[1, 0]) + 1.5 * max(rp))])
        #plt.ylim(np.array([-1, 1]) * (abs(x0[1, 0]) + 1.5 * max(rp)))
        plt.draw()
        plt.draw()
        plt.pause(0.01)

# Output variables
h = np.linalg.norm(x[:, 1, :] - x[:, 0, :], axis=0) - np.sum(rp)  # gap height, m
vr = np.linalg.norm(v[:, 0, :] - v[:, 1, :], axis=0)              # relative velocity, m/s

xCoM = np.sum(Mp[:, np.newaxis] * x, axis=1) / np.sum(Mp)         # center of mass position, m
vCoM = np.sum(Mp[:, np.newaxis] * v, axis=1) / np.sum(Mp)         # center of mass velocity, m/s

ECoM = 1 / 2 * np.sum(Mp) * np.linalg.norm(vCoM, axis=0)**2       # center of mass kinetic energy, J
Ekin = 1 / 2 * Mp[:, np.newaxis] * np.linalg.norm(v - vCoM[:, np.newaxis, :], axis=0)**2  # particle kinetic energy relative to center of mass, J
Erot = 1 / 2 * (2 / 5 * Mp[:, np.newaxis] * rp[:, np.newaxis]**2) * omega**2  # rotational energy, J
Etot = np.sum(Ekin, axis=0) + np.sum(Erot, axis=0) + Epot         # total energy, J

e = vr[-1] / vr[0]                                                # total restitution coefficient, -
en = abs((v[0, 0, -1] - v[0, 1, -1]) / (v[0, 0, 0] - v[0, 1, 0]))  # normal restitution coefficient, -
et = abs((v[1, 0, -1] - v[1, 1, -1]) / (v[1, 0, 0] - v[1, 1, 0]))  # tangential restitution coefficient, -

thetai0 = np.arctan(v[1, 0, 0] / v[0, 0, 0])  # impact angle i, rad
thetaj0 = 0                                   # impact angle j, rad
thetai1 = np.arctan(v[1, 0, -1] / v[0, 0, -1])  # rebound angle i, rad
thetaj1 = np.arctan(v[1, 1, -1] / v[0, 1, -1])  # rebound angle j, rad

Sigma = abs(thetaj1 - thetai1) / (np.pi / 2)   # effective rebound angle, -

# Display results
print(f'St = {St:.4f}; theta0 = {np.rad2deg(theta0):.2f} deg; e = {e:.2f}; en = {en:.2f}; et = {et:.2f}; Sigma = {Sigma:.2f}')