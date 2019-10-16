import numpy as np
import scipy.interpolate as interp
import matplotlib.pyplot as plt
plt.style.use('bmh')    # Nicer looking plots

# Properties of the rolling object
r = 0.016                # m      (radius)
g = 9.81                 # m/s^2  (gravitational acceleration)

m = 0.0276               # kg     (mass)
c = 2/5
I0 = c*m*r**2            # kg m^2 (moment of inertia)

# Properties of the frame
L = 1.4                   # m (length)
yi = [0.640,0.465,0.327,0.208,0.117,0.049,0.014,0.0]  # m (y-positions)

N = len(yi)               #   (# of mounts)
xi = np.linspace(0, L, N) # m (x-positions)

# Callable class for the track curve
get_y = interp.CubicSpline(xi, yi, bc_type="natural")

get_dydx = get_y.derivative()  # Callable derivative of track
get_d2ydx2 = get_dydx.derivative()  # Callable double derivative of track


def get_theta(x):
    """ Returns the angle of the track. """
    return -np.arctan(get_dydx(x))


def get_R(x):
    """ Returns the radius of the curvature. """
    return (1 + (get_dydx(x)) ** 2) ** 1.5 / get_d2ydx2(x)


def get_curvature(x):
    """ Returns the curvature (1/R). """
    return get_d2ydx2(x) / (1 + (get_dydx(x)) ** 2) ** 1.5

x = np.linspace(xi[0], xi[-1], 200)

# Create figure
fig, axarr = plt.subplots(3, 1, sharex=True, figsize=(12, 9), dpi=100)
fig.subplots_adjust(hspace=0.02)

# Axes 1:
axarr[0].plot(x, get_y(x), 'C0', label=r"$y(x)$")
axarr[0].plot(xi, yi, 'C1o', label="Mounts")
axarr[0].set_ylabel(r"$y(x)$, [m]", size='15')
#axarr[0].set_aspect('equal')

# Axes 2:
axarr[1].plot(x, get_theta(x), 'C0')
axarr[1].set_ylabel(r"$\theta(x)$, [rad]", size='15')

# Axes 2:
axarr[2].plot(x, get_curvature(x), 'C0')
axarr[2].set_ylabel(r"$\kappa(x)$, [1/m]", size='15')

plt.show()
