# 3-Body Problem - Devadarsh Arangaly Vijesh

import numpy as num
import matplotlib.pyplot as plt
plt.switch_backend('TkAgg')

# gravitational constant
G = 6.67430e-11

# represents a celestial body
class Body:
    def __init__(self, mass, position, velocity):
        self.mass = mass
        self.position = num.array(position, dtype = float)
        self.velocity = num.array(velocity, dtype = float)
        self.force = num.array([0.0,0.0], dtype = float)

# creating bodies
body1 = Body(mass=1.0e26, position=[-1.0e11, 0], velocity=[0, 1.0e3])
body2 = Body(mass=1.0e26, position=[1.0e11, 0], velocity=[0, -1.0e3])
body3 = Body(mass=1.0e26, position=[0, 1.0e11], velocity=[-1.0e3, 0])
bodies = [body1, body2, body3]

# computes gravitational force between the bodies
def computeForce(body1, body2):
    r = body2.position - body1.position
    distance = num.linalg.norm(r)
    
    if distance == 0:
        return num.array([0.0, 0.0])
    
    magnitude = G * body1.mass * body2.mass / distance**2
    direction = r / distance
    return magnitude * direction

# updates forces on all bodies
def updateForce(bodies):
    for body in bodies:
        body.force[:] = 0.0

    for i, body1 in enumerate(bodies):
        for j, body2 in enumerate(bodies):
            if i != j:
                body1.force += computeForce(body1, body2)

# updates positons and velocities
def updatePositionAndVelocity(bodies, dt):
    for body in bodies:
        body.velocity += body.force / body.mass * dt
        body.position += body.velocity * dt

# simulation parameters
dt = 1000
numSteps = 10000

# positions for plotting
positions = {body: [] for body in bodies}

# running simulation
for step in range(numSteps):
    updateForce(bodies)
    updatePositionAndVelocity(bodies, dt)

    # storing current positions
    for body in bodies:
        positions[body].append(body.position.copy())

# converting positions to numpy arrays for plotting
for body in positions:
    positions[body] = num.array(positions[body])

# plotting
for body, pos in positions.items():
    plt.plot(pos[:, 0], pos[:, -1])

# adjusting the plot
colors = ['r', 'g', 'b']  # Colors for each body
labels = ['Body 1', 'Body 2', 'Body 3']  # Labels for the bodies
plt.figure(figsize=(10,8))

for i, (body, pos) in enumerate(positions.items()):
    plt.plot(pos[:, 0], pos[:, 1], color=colors[i], label=labels[i])

plt.xlabel('Position (m)')
plt.ylabel('Position (m)')
plt.title('Three-Body Simulation')
plt.grid(True)
plt.legend()
plt.show()

# making an animation
from matplotlib.animation import FuncAnimation
fig, ax = plt.subplots(figsize=(10,8))
colors = ['r', 'g', 'b']
lines = [ax.plot([], [], color=colors[i])[0] for i in range(3)]

ax.set_xlim(-2e11, 2e11)
ax.set_ylim(-2e11, 2e11)
ax.set_xlabel('X Position (m)')
ax.set_ylabel('Y Position (m)')
ax.set_title('Three-Body Problem Simulation')

def update(frame):
    for i, body in enumerate(bodies):
        lines[i].set_data(positions[body][:frame, 0], positions[body][:frame, 1])
    return lines

ani = FuncAnimation(fig, update, frames=numSteps, interval=50, blit = True)
plt.show()





