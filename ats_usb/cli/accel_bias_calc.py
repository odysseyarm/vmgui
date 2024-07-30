import pprint
import json
import argparse
from sympy import Symbol, Eq, nsolve

# Define symbols
b_x, b_y, b_z, s_x, s_y, s_z = map(Symbol, "b_x b_y b_z s_x s_y s_z".split())

# Acceleration measurements
x_x, x_y, x_z = [0.13028911, 0.12664859, 9.98692131]
y_x, y_y, y_z = [-9.50260544, 0.07374427, -2.35186720]
z_x, z_y, z_z = [0.17701271, 7.17228413, -6.64125538]
a_x, a_y, a_z = [8.29341698, 4.16669178, -3.09963512]
c_x, c_y, c_z = [3.54227853, 5.16701508, 7.60187197]
d_x, d_y, d_z = [2.97791672, -2.96459055, 9.08387566]

# Gravity in Houston
g = 9.79281

# Define equations using Eq
eq1 = Eq((s_x * x_x + b_x)**2 + (s_y * x_y + b_y)**2 + (s_z * x_z + b_z)**2, g**2)
eq2 = Eq((s_x * y_x + b_x)**2 + (s_y * y_y + b_y)**2 + (s_z * y_z + b_z)**2, g**2)
eq3 = Eq((s_x * z_x + b_x)**2 + (s_y * z_y + b_y)**2 + (s_z * z_z + b_z)**2, g**2)
eq4 = Eq((s_x * a_x + b_x)**2 + (s_y * a_y + b_y)**2 + (s_z * a_z + b_z)**2, g**2)
eq5 = Eq((s_x * c_x + b_x)**2 + (s_y * c_y + b_y)**2 + (s_z * c_z + b_z)**2, g**2)
eq6 = Eq((s_x * d_x + b_x)**2 + (s_y * d_y + b_y)**2 + (s_z * d_z + b_z)**2, g**2)

# Solve equations
solution = nsolve((eq1, eq2, eq3, eq4, eq5, eq6), (b_x, b_y, b_z, s_x, s_y, s_z), (0, 0, 0, 1, 1, 1))

# Extract solution into a dictionary
solution_dict = {
    "b_x": float(solution[0]),
    "b_y": float(solution[1]),
    "b_z": float(solution[2]),
    "s_x": float(solution[3]),
    "s_y": float(solution[4]),
    "s_z": float(solution[5])
}

# Pretty print the solution
pprint.pprint(solution_dict)

# Argument parser
parser = argparse.ArgumentParser(description='Save solution to a file.')
parser.add_argument('output_file', type=str, help='Output file to save the solution')
args = parser.parse_args()

# Save solution to a file
with open(args.output_file, 'w') as f:
    json.dump(solution_dict, f, indent=4)
