from sympy import Symbol, nsolve
x_x, x_y, x_z, \
y_x, y_y, y_z, \
z_x, z_y, z_z, \
b_x, b_y, b_z, \
g = map(Symbol, "x_x x_y x_z y_x y_y y_z z_x z_y z_z b_x b_y b_z g".split())


# 3 accel measurements
x_x, x_y, x_z = [0.29391649, 0.80576897, 9.93242264]
y_x, y_y, y_z = [9.50738144, -0.17829479, -2.44243479]
z_x, z_y, z_z = [-1.26677823, 9.09933949, 3.32616949]

# Gravity in Houston determined by https://www.ngs.noaa.gov/cgi-bin/grav_pdx.prl
g = 9.79281

eq1 = (x_x + b_x)**2 + (x_y + b_y)**2 + (x_z + b_z)**2 - g**2
eq2 = (y_x + b_x)**2 + (y_y + b_y)**2 + (y_z + b_z)**2 - g**2
eq3 = (z_x + b_x)**2 + (z_y + b_y)**2 + (z_z + b_z)**2 - g**2

s = nsolve([eq1, eq2, eq3], [b_x, b_y, b_z], [0, 0, 0], dict=True)
print(s)
