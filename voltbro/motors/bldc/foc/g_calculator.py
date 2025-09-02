import numpy as np

T = 0.00005

# TODO: move all of this to C++ code as a constexpr function
expected_rps = 2
p0 = w = (np.pi*2 * 5) * expected_rps
# angle (should be [40;50])
theta_deg = 45
ang = np.deg2rad(theta_deg)

ro0 = np.exp(-p0 * T)
ro1 = np.exp(-w * T * np.cos(ang))
phi = w * T * np.sin(ang)

c2 = -2 * ro1 * np.cos(phi) - ro0
c1 = ro1 * (2*ro0*np.cos(phi) + ro1)
c0 = -ro0 * ro1 * ro1

l1 = c2 + 3
l2 = (c1 - c0 - 4 + 3*l1) / (2*T)
l3 = (c1 + c0 - 2 + l1) / (np.square(T))

g3 = l3
g2 = l2 - T*g3
g1 = l1 - T*g2 - g3*np.square(T)/2

print(g1, g2, g3)
