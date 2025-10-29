import numpy as np
import matplotlib.pyplot as plt
import matplotlib.tri as tri
import csv

def design_matrix(xy, degree=3):
    x = xy[:, 0]
    y = xy[:, 1]
    if degree == 2:
        return np.column_stack([np.ones(len(x)), x, y, x**2, x*y, y**2])
    elif degree == 3:
        return np.column_stack([
            np.ones(len(x)), x, y, x**2, x*y, y**2,
            x**3, x**2*y, x*y**2, y**3
        ])

with open('ffmap.csv', 'r') as file:
    csv_reader = csv.reader(file)
    data_list = list(csv_reader)

data_array = np.array(data_list, dtype=float)

ee_pos = data_array[:, 0:2]
tensions = data_array[:, 2:6]
error = data_array[:, 6]

weights = np.ones(len(error))
degree = 3

for i, e in enumerate(error):
    if e <= 0.02:
        weights[i] = 1.0
    elif e <= 0.03:
        weights[i] = 0.8
    elif e <= 0.04:
        weights[i] = 0.6
    elif e <= 0.05:
        weights[i] = 0.4
    elif e <= 0.06:
        weights[i] = 0.2
    else:
        weights[i] = 0.1

coeff_list = []
A = design_matrix(ee_pos, degree) # Nx10 for cubic
for i in range(4):
    b = tensions[:, i]                   # T1 for example
    W = np.diag(weights)                  # NxN diagonal weight matrix

    # Solve weighted least squares: a = (A^T W A)^(-1) A^T W b
    a = np.linalg.inv(A.T @ W @ A) @ (A.T @ W @ b)

    coeff_list.append(a)

coeffs = np.asarray(coeff_list)

predicted = np.empty_like(tensions)
for i in range(4):
    predicted[:, i] = A @ coeffs[i]   # predicted tension for cable i

# Plot scatter for each cable
cable_labels = ['T0', 'T1', 'T2', 'T3']
colors = ['r', 'g', 'b', 'm']

plt.figure(figsize=(8, 8))
for i in range(4):
    plt.scatter(tensions[:, i], predicted[:, i], label=cable_labels[i], alpha=1.0, color=colors[i], s=10)

# pred vs measured reference line
min_val = min(tensions.min(), predicted.min())
max_val = max(tensions.max(), predicted.max())
plt.plot([min_val, max_val], [min_val, max_val], 'k-', label='predicted = measured')

plt.xlabel('Measured Tension')
plt.ylabel('Predicted Tension')
if degree == 2:
    plt.title('Measured vs Predicted Tensions (2nd Order Polynomial)')
elif degree ==3:
    plt.title('Measured vs Predicted Tensions (3nd Order Polynomial)')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.show()

tension_errors = np.abs(tensions - predicted)

rmse_per_cable = np.sqrt(np.mean(tension_errors**2, axis=0))
for i, cable in enumerate(cable_labels):
    print(f'RMSE {cable}: {rmse_per_cable[i]:.4f} N')

positions = {
    0: (0, 1),  # top-right
    1: (1, 1),  # bottom-right
    2: (0, 0),  # top-left
    3: (1, 0)   # bottom-left
}

# plt.figure(figsize=(12, 10))
# for i in range(4):
#     plt.subplot(2, 2, i+1)
#     sc = plt.scatter(ee_pos[:, 0], ee_pos[:, 1], c=tension_errors[:, i], cmap='hot', s=50)
#     plt.colorbar(sc, label=f'{cable_labels[i]} Absolute Error (N)')
#     plt.xlabel('X Position (m)')
#     plt.ylabel('Y Position (m)')
#     plt.title(f'{cable_labels[i]} Error Map')
#     plt.axis('equal')
#     plt.grid(True)

# plt.tight_layout()
# plt.show()

plt.figure(figsize=(12, 10))

for i in range(4):
    row, col = positions[i]
    ax = plt.subplot2grid((2, 2), (row, col))
    
    # Triangulation for scattered points
    triang = tri.Triangulation(ee_pos[:, 0], ee_pos[:, 1])
    
    # Filled contour plot
    cont = ax.tricontourf(triang, tension_errors[:, i], levels=20, cmap='hot')
    plt.colorbar(cont, ax=ax, label=f'{cable_labels[i]} Absolute Error (N)')
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_title(f'{cable_labels[i]} Error Heatmap')
    ax.set_aspect('equal')
    ax.grid(True)

plt.tight_layout()
plt.show()