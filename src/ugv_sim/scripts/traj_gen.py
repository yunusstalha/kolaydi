import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

# Define start and end positions
start_position = [0, 0]  # Starting coordinates
end_position = [10, 10]  # Ending coordinates

# Define the times at which the start and end positions occur
times = [0,1,2,3,4,5,6]  # Start at time 0, end at time 1

# Create arrays for x and y coordinates
x = [0,1,3,2,2,3,1]
y = [0,1,-2,-2,5,10,10]

# Create cubic splines with boundary conditions (zero velocity at start and end)
cs_x = CubicSpline(times, x, )
cs_y = CubicSpline(times, y, )

# Sample points from the cubic spline
t_new = np.linspace(0, 6, 100)
tt_new = np.linspace(0,6,100)
x_new = cs_x(t_new)
y_new = cs_y(tt_new)


xy_new = np.column_stack((x_new, y_new))


print(x_new, y_new)
# Plot the trajectory
plt.plot(x_new, y_new)
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Robot Trajectory')
plt.grid(True)
plt.show()
