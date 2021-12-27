# PotentialFieldPlanner

*dynamicObs and plotmap p files are from instructors at the University of Pennsylvania; their underlying m files were not written by me. 

Function Description:

**potential_field_step**

Potential field step takes in the robot’s current configuration, the map struct (which contains the obstacle positions and goal configuration). Taking the position of the robot, obstacles, and goal, it calculates artificial forces acting on the robot, generating such forces for every joint on the robot. It then turns those forces into artificial robot torques (or joint efforts). To do this, it generates 6 joint torques for an artificial force on the end effector (the 6th joint), 5 joint torques for an artificial force on the 5th joint (acting on joints 1-5), 4 joint torques for an artificial force on the 4th joint (acting on joints 1-4), etc. This populates a 6 by 6 matrix of joint torques; the code then collapses this matrix into a 1x6 torque vector by adding up all the individual joint torques. For example, all 6 joint torques acting on joint 1 get added, the 5 joint torques acting on joint 2 get added, etc.
The robot then uses these joint torques to execute gradient descent in the potential field. If the robot is close enough (determined by threshold tolToGoal) to the goal, the robot completes its calculation and throws the isDone flag. However, if the robot is not sufficiently close to the goal, and the gradient descent step generates a step size that is less than threshold epsilon for at least three steps, we determine the robot to be stuck in a local minimum. To get out of the local minimum, the robot executes a random walk, where all six joint configuration variables are perturbed by a small random value pulled from a normal distribution with standard deviation v. The code checks to see whether or not each step in the random walk will generate a collision with one of the obstacles; if it doesn’t, the code executes the random walk.

**calcJacobian_11**

This function generates a jacobian based on the robot’s current configuration and the joint that the robot is planning on.

**forceToTorque_11**

This function is simple; it generates torques from forces via the Jacobian.
