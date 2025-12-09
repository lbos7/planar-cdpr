import numpy as np
import matplotlib.pyplot as plt
import csv

with open('fk_apriltag_data/fk_apriltag_scatter.csv', 'r') as file:
    csv_reader = csv.reader(file)
    data_list = list(csv_reader)

data_array = np.array(data_list, dtype=float)

# Split into the two XY positions
fk_array = data_array[:, 0:2]  # (x1, y1)
apriltag_array = data_array[:, 2:4]  # (x2, y2)

# Compute Euclidean distance row-by-row
errors = np.linalg.norm(apriltag_array - fk_array, axis=1)
print(f"Max error: {np.max(errors):.4f}")
print(f"Min error: {np.min(errors):.4f}")
print(f"Average error: {np.mean(errors):.4f}")
print(f"Median error: {np.median(errors):.4f}")

plt.figure(figsize=(6, 6))
plt.scatter(fk_array[:, 0], fk_array[:, 1], s=10, c='r')
plt.scatter(apriltag_array[:, 0], apriltag_array[:, 1], s=10, c='b')
plt.legend(["FK Estimated Pos", "Apriltag Estimated Pos"])
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.title(f"FK and Apriltag Position Estimation")
plt.axis('equal')
plt.show()