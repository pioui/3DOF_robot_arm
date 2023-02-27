# 3DOF_robot_arm

## Kinematic simulation of a linear trajectory for a 3 DOF Robot Arm Manipulator using MATLAB

### DH parameters of the Robotic arm
i  | $θ_i$  | $d_i$ 	 | $a_i$	| $α_i$ 
---  | ---  | ---      | ---         | ---  
0' | $0$    | $l_0$   	 | $0$    	| $90$   
1  | $q_1$    | $0$ 		 | $0$  	| $-90$    
2  | $q_2$    | $l_1$ 	| $l_2$   	| $0$     
E  | $q_3$  | $0$   	| $l_3$	| $0$   


### Direct and Inverse kinematics

$ R^0_E =  	\begin{bmatrix}   c_1c_2c_3 - c_1s_2s_3 & - c_1c_2s_3 - c_1c_3s_2 & -s_1 \\
                                                        	c_2s_3 + c_3s_2 &   c_2c_3 - s_2s_3 &  0 \\
                                                        	c_2c_3s_1 - s_1s_2s_3 & - c_2s_1s_3 - c_3s_1s_2 &  c_1 \\ \end{bmatrix} \\ $
