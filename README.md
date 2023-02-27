# 3DOF_robot_arm

## Kinematic simulation of a linear trajectory for a 3 DOF Robot Arm Manipulator using MATLAB

Robotics I - ECE NTUA 

### 1. Kinematic Analysis

#### DH parameters of the Robotic arm
i  | $θ_i$  | $d_i$ 	 | $a_i$	| $α_i$ 
---  | ---  | ---      | ---         | ---  
0' | $0$    | $l_0$   	 | $0$    	| $90$   
1  | $q_1$    | $0$ 		 | $0$  	| $-90$    
2  | $q_2$    | $l_1$ 	| $l_2$   	| $0$     
E  | $q_3$  | $0$   	| $l_3$	| $0$   

#### Inverse and Direct Kinematics Model
Using the DH parameters we can easily calculate the Direct Kinematics Model and the Jacobian as describes in direct_kinematics.m. After some mathematical analysis we also calculate the Inverse Kinematics Model.

### 2. Trajectory Planning

In this part we are asked to implement the simulation of a periodic rectilinear displacement of the finite element of the arm we studied. This movement is smooth between the two extreme positions $ p_A = (x_A , y_A , h ) $ and $ p_B = (x_B , y_B ,h ) $ on a horizontal plane, distance $ h$ from the center of the robotic reference system basis. The final action element starts from a stop at point $ p_A $ at time $ t=0$ and reaches point $ p_B$ at time $t=T sec$. The line joining these two points is: $y = \frac{y_B-y_A}{x_B-x_A}(x-x_A)+y_A = f (x) $. So for the trajectory coordinates we have $(x,y,z) = (x, f(x), h)$.

So we will need to draw the trajectory $x(t)$. We choose to use the cubic polynomial method. We did not choose a polynomial of higher degree or a function with parabolic mixing because for the movement we need to perform we do not need to calculate both the velocity and the acceleration at the ends of the segment and it is easier and more economical to implement.


So the desired trajectories will have the following form:

$ x(t) = a_{0x}+a_{1x}t+a_{2x}t^2+a_{3x}t^3 $

### 3. Simulation

Using the MATLAB ($ linear-movement.m$) we simulate the movement from the point $P_A = (3,3,6)$ to $ P_B = (4,2,6)$ in a time interval $ T=10sec $ of the given robot with dimensions $l_0=3, l_1=2, l_2=3, l_3=2$. 

First, we calculate the desired trajectories and velocities 
![alt text](http://url/to/img.png)
Then we choose one of the inverse kinematics solutions that we calculated in part 1 and apply it to find the angles of the joints. Finally, we apply the direct kinematic formulas that we calculated in part 1 to calculate the intermediate positions of the joints and also of the final element as seen in the diagram.
![alt text](http://url/to/img.png)

