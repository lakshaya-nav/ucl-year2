import numpy as np
import matplotlib.pyplot as plt

def create_points_sphere(radius, n):
    u = np.random.rand(n)
    v = np.random.rand(n)

    theta = 2 * np.pi * u  # uniform in [0, 2π)
    phi = np.arccos(v)  # weighted correctly for area
    x = []
    y = []
    z = []
    for i in range(n):
        x.append(radius * np.sin(phi[i]) * np.cos(theta[i]))
        y.append(radius * np.sin(phi[i]) * np.sin(theta[i]))
        z.append(radius * np.cos(phi[i]))

    return x,y,z

x_1,y_1,z_1 = create_points_sphere(1, 200)
x_2,y_2,z_2 = create_points_sphere(1.5, 200)


fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x_1, y_1, z_1, s=5, c='blue', alpha=0.6)
ax.scatter(x_2, y_2, z_2, s=5, c='red', alpha=0.6)

# Add labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Uniformly Distributed Points on a Sphere')

plt.show()



