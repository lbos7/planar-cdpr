import numpy as np
import matplotlib.pyplot as plt
import csv

with open('fk_apriltag_data/fk_apriltag_scatter1.csv', 'r') as file:
    csv_reader = csv.reader(file)
    data_list = list(csv_reader)

data_array = np.array(data_list, dtype=float)

with open('fk_apriltag_data/fk_apriltag_scatter_passive.csv', 'r') as passive_file:
    csv_reader_passive = csv.reader(passive_file)
    data_list_passive = list(csv_reader_passive)

data_array_passive = np.array(data_list_passive, dtype=float)

# Split into the two XY positions
fk_array = data_array[:, 0:2]  # (x1, y1)
apriltag_array = data_array[:, 2:4]  # (x2, y2)

# Split into the two XY positions
fk_array_passive = data_array_passive[:, 0:2]  # (x1, y1)
apriltag_array_passive = data_array_passive[:, 2:4]  # (x2, y2)

# Compute Euclidean distance row-by-row
errors = np.linalg.norm(apriltag_array - fk_array, axis=1)
print(f"Max error: {np.max(errors):.4f}")
print(f"Min error: {np.min(errors):.4f}")
print(f"Average error: {np.mean(errors):.4f}")
print(f"Median error: {np.median(errors):.4f}\n")

# Compute Euclidean distance row-by-row
errors_passive = np.linalg.norm(apriltag_array_passive - fk_array_passive, axis=1)
print(f"Passive EE Max error: {np.max(errors_passive):.4f}")
print(f"Passive EE Min error: {np.min(errors_passive):.4f}")
print(f"Passive EE Average error: {np.mean(errors_passive):.4f}")
print(f"Passive EE Median error: {np.median(errors_passive):.4f}")

plt.figure(figsize=(6, 6))
plt.scatter(fk_array[:, 0], fk_array[:, 1], s=10, c='r')
plt.scatter(apriltag_array[:, 0], apriltag_array[:, 1], s=10, c='b')
plt.legend(["FK Estimated Pos", "Apriltag Estimated Pos"])
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.title(f"FK and Apriltag Position Estimation")
plt.axis('equal')
plt.show()

plt.figure(figsize=(6, 6))
plt.scatter(fk_array_passive[:, 0], fk_array_passive[:, 1], s=10, c='r')
plt.scatter(apriltag_array_passive[:, 0], apriltag_array_passive[:, 1], s=10, c='b')
plt.legend(["FK Estimated Pos", "Apriltag Estimated Pos"])
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.title(f"FK and Apriltag Position Estimation for Passive EE")
plt.axis('equal')
plt.show()

plt.figure(figsize=(6, 6))
plt.scatter(fk_array[:, 0], fk_array[:, 1], s=5, c='r')
plt.scatter(apriltag_array[:, 0], apriltag_array[:, 1], s=5, c='b')
plt.scatter(fk_array_passive[:, 0], fk_array_passive[:, 1], s=5, c='g')
plt.scatter(apriltag_array_passive[:, 0], apriltag_array_passive[:, 1], s=5, c='m')
plt.legend(["FK Estimated Pos", "Apriltag Estimated Pos", "FK Estimated Pos (Passive EE)", "Apriltag Estimated Pos (Passive EE)"])
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.title(f"FK and Apriltag Position Estimation")
plt.axis('equal')
plt.show()