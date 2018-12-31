# 3R_motion
**3R motion with Scilab**
See the TriVex SL/SLi vertical pack loader using two 3R arm robots: https://youtu.be/58p0063SkLE
Using the functions developed at class, finish the Scilab code to generate the vertical motion of one of the robots. The code should generate the following results:
1 - Plot of the 3R motion
2 - Plot of joint speeds versus time. For this, you can use the following sequence at each step of the motion:
-
Compute the pose of the end-effector
-
Solve the inverse position analysis to obtain the configuration of the robot and, in particular, the position of each joint
-
Compute the jacobian at this configuration
-
Obtain joint speeds by multiplying the jacobian with the required end-effector twist (joint speeds = J·T)

Optionally (as bonus) you can generate the following results:
3 - Plot the torque that J1 must sustain versus time (you can consider the weight of L1 and L2 at the middle of the link, and the weight of the end-effector at the middle
of g3)
4 - In the function that solves the inverse position analysis of the robot, add a check to guarantee that the inputs are inside the workspace of the robot
