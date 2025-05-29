import matplotlib.pyplot as plt
import csv

robot_x = []
robot_y = []
estimate_x = []
estimate_y = []

with open('build/trajectory_log.csv', newline='') as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        robot_x.append(float(row['robot_x']))
        robot_y.append(float(row['robot_y']))
        estimate_x.append(float(row['estimate_x']))
        estimate_y.append(float(row['estimate_y']))

plt.figure(figsize=(8, 8))
plt.plot(robot_x, robot_y, label='Robot True Position', marker='o')
plt.plot(estimate_x, estimate_y, label='Particle Filter Estimate', marker='x')
plt.title('Robot Trajectory vs Particle Filter Estimate')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.legend()
plt.grid(True)
plt.axis('equal')

plt.savefig('build/trajectory_plot.png')  # Save the plot as PNG in build/
plt.show()
