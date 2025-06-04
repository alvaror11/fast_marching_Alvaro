from sklearn.model_selection import ParameterSampler
import numpy as np
import os

# Parameter ranges definition
param_grid = {
    'start_x': np.linspace(5.0, 45.0, 10),
    'start_y': np.linspace(5.0, 45.0, 10),
    'objective_x': np.linspace(5.0, 45.0, 10),
    'objective_y': np.linspace(5.0, 45.0, 10),
    'step': [0.5],
    'distance_threshold': np.linspace(4.0, 10.0, 3),
    'escalado_vectores': np.linspace(4.0, 6.0, 3),
    'flight_level': np.linspace(70.0, 230.0, 3),
    'planner_type': np.linspace(0.0, 2.0, 3)
}

# Number of combinations you want
n_combinations = 50

# Generate random combinations
combinations = list(ParameterSampler(param_grid, n_combinations))

print("\nParameter Combinations Generated:")
print("-" * 100)
print(f"{'Combination':^12} | ", end="")
for key in param_grid.keys():
    print(f"{key:^8} | ", end="")
print("\n" + "-" * 100)

for i, params in enumerate(combinations, 1):
    print(f"{i:^12} | ", end="")
    for key in param_grid.keys():
        print(f"{params[key]:8.2f} | ", end="")
    print()

print("-" * 100)
print(f"\nTotal combinations generated: {n_combinations}")
# Save to file

current_dir = os.path.dirname(os.path.abspath(__file__))  # Gets Src directory
parent_dir = os.path.dirname(current_dir)  # Gets functions directory
archivos_dir = os.path.join(parent_dir, "Archivos")
output_file = os.path.join(archivos_dir, "parameter_combinations.txt")

print(f"Current directory: {current_dir}")
print(f"Parent directory: {parent_dir}")
print(f"Archivos directory: {archivos_dir}")

with open(output_file, 'w') as f:
    # Write header
    f.write(f"{len(combinations)} {len(param_grid)}\n")
    f.write(" ".join(param_grid.keys()) + "\n")
    
    # Write combinations
    for params in combinations:
        values = [f"{params[key]:.2f}" for key in param_grid.keys()]
        f.write(" ".join(values) + "\n")

print(f"Generated {len(combinations)} parameter combinations")
print(f"Saved to: {output_file}")