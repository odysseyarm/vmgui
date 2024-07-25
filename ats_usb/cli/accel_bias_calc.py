from sympy import Symbol, nsolve
b_x, b_y, b_z, s_x, s_y, s_z, g = map(Symbol, "b_x b_y b_z s_x s_y s_z g".split())


# 6 accel measurements
x_x, x_y, x_z = [0.13028911, 0.12664859, 9.98692131]
y_x, y_y, y_z = [-9.50260544, 0.07374427, -2.35186720]
z_x, z_y, z_z = [0.17701271, 7.17228413, -6.64125538]
a_x, a_y, a_z = [8.29341698, 4.16669178, -3.09963512]
c_x, c_y, c_z = [3.54227853, 5.16701508, 7.60187197]
d_x, d_y, d_z = [2.97791672, -2.96459055, 9.08387566]

# Gravity in Houston determined by https://www.ngs.noaa.gov/cgi-bin/grav_pdx.prl
g = 9.79281

eq1 = (s_x*x_x + b_x)**2 + (s_y*x_y + b_y)**2 + (s_z*x_z + b_z)**2 - g**2
eq2 = (s_x*y_x + b_x)**2 + (s_y*y_y + b_y)**2 + (s_z*y_z + b_z)**2 - g**2
eq3 = (s_x*z_x + b_x)**2 + (s_y*z_y + b_y)**2 + (s_z*z_z + b_z)**2 - g**2
eq4 = (s_x*a_x + b_x)**2 + (s_y*a_y + b_y)**2 + (s_z*a_z + b_z)**2 - g**2
eq5 = (s_x*c_x + b_x)**2 + (s_y*c_y + b_y)**2 + (s_z*c_z + b_z)**2 - g**2
eq6 = (s_x*d_x + b_x)**2 + (s_y*d_y + b_y)**2 + (s_z*d_z + b_z)**2 - g**2

s = nsolve([eq1, eq2, eq3, eq4, eq5, eq6], [b_x, b_y, b_z, s_x, s_y, s_z], [0, 0, 0, 1, 1, 1], dict=True)
print(s)
