# Quadrupled_Robot
Project Overview:

I've developed a quadruped robot simulation using Python along with several libraries. The goal of the project is to simulate a four-legged robot (similar to a lion or dog) in a virtual environment using the PyBullet physics engine. The focus is on controlling the robot's movement using inverse kinematics (IK), which helps it walk smoothly by calculating the required joint angles.

Technologies & Libraries Used:
Python: This is the main programming language I used for the entire project.
PyBullet: A powerful physics simulation library that provides an interface to create, control, and visualize 3D objects. I used it to:
Generate planes and the quadruped robot body.
Handle the physics and collisions to simulate realistic movements.
Numpy:
I used Numpy for working with arrays and performing mathematical calculations, especially for matrix operations and trigonometric functions needed in the IK equations.
Time:
The time library is used to control the frequency of frame updates based on keyboard inputs. This helps in making the robot’s movements appear smooth and responsive.
How the Project Works:
Inverse Kinematics Calculation:

The heart of this project is the K_solver.py file, where I implemented the inverse kinematics calculations. The IK equations help determine the joint angles required to move each leg to a specified target position in 3D space.
By calculating angles for the hip, knee, and ankle joints, I can control how each leg moves, allowing the robot to walk forward, backward, or even rotate.
Coordinate System in PyBullet:

PyBullet uses a 3D coordinate system where:
The Z-axis points upward.
The X-axis represents the forward direction.
The Y-axis is sideways.
I used this system to place the robot and define its movement.
Using Numpy for Calculations:

For solving the IK equations, Numpy helped me efficiently handle complex mathematical operations, like solving trigonometric equations and working with arrays.
Frame Control with Time Library:

I used the time module to adjust the frequency of frame updates when I press keyboard keys to control the robot’s speed and direction. This allows me to interact with the simulation in real-time.
Simulation in PyBullet:

PyBullet provides a realistic simulation environment where I can visualize how the quadruped robot moves based on the joint angles calculated by the IK solver. The library also handles the rendering of objects like planes and the quadruped model itself.
Real-world Application: This project can be a stepping stone for developing real quadruped robots for various applications, such as exploration, rescue missions, or even delivery robots that can navigate rough terrains where wheeled robots might struggle.

Future Enhancements:

I plan to add more sophisticated control algorithms for smoother movements.
In the future, I want to explore using reinforcement learning so the robot can learn to walk on its own in different environments.
