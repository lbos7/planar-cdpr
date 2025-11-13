import numpy as np
import matplotlib.pyplot as plt
import csv

with open('waypoint_data/ff_traj_square25_passive.csv', 'r') as file:
# with open('waypoint_data/ff_traj_square25.csv', 'r') as file:
# with open('waypoint_data/traj_square2.csv', 'r') as file:
    csv_reader = csv.reader(file)
    data_list = list(csv_reader)


data_array = np.array(data_list, dtype=float)
waypoints = np.vstack([data_array[:4, :], data_array[0, :]])
speed = data_array[4, 0]
thresh = data_array[4, 1]
ee_pos_array = data_array[5:]

using_ff = "ff" in file.name
passive = "passive" in file.name

plt.figure(figsize=(8, 8))
plt.plot(waypoints[:, 0], waypoints[:, 1], '-ok')
plt.plot(ee_pos_array[:, 0], ee_pos_array[:, 1], '-r')

# Draw circle outlines around each waypoint
for wp in waypoints:
    circle = plt.Circle((wp[0], wp[1]), radius=thresh, color='b', fill=False, linestyle='--', linewidth=1)
    plt.gca().add_patch(circle)

plt.legend(["Ideal Path", "Estimated EE Pos", "Waypoint Threshold"])
plt.xlabel("x (m)")
plt.ylabel("y (m)")
if using_ff and passive:
    plt.title(f"Passive EE Movement Performance (FK Estimate) - Trajectory Control w/ Feedforward @ {speed} m/s")
elif using_ff and not passive:
    plt.title(f"EE Movement Performance (FK Estimate) - Trajectory Control w/ Feedforward @ {speed} m/s")
else:
    plt.title(f"EE Movement Performance (FK Estimate) - Trajectory Control @ {speed} m/s")
plt.axis('equal')
plt.show()