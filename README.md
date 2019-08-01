# Trajectory_Optimization_of_2-link_Robotic_Arm
This optimization project aims to find the best trajectory of a 2-link robotic arm to achieve minimum joint torque over time when lifting objects up from ground

This project was completed in May 02, 2019, and was just uploaded to my Github account recently.

In this optimization problem, we want to minimize the energy cost of the robotic arm when manipulating an item. Therefore, the proposed optimization problem is to minimize the time integral of torque on each joint throughout the entire trajectory with respect to the arm’s global angular position, velocity and torque at every time point subject to the physical constraints of the robotic arm.

The dynamic equation was derived and implemented in DynamicEquation.m, and the constrained optimization problem was solved using MATLAB fmincon() function with both natural and pratical constraint. Finally, the optimization result was visualized in animate.m

### Sample optimized trajectory of the robotic arm under different object mass (time duration = 1s): ###
Optimized trajectory with the object being equally heavy as the arm  
(mass of arm 1 = 1kg, mass of arm 2 = 1kg, mass of object = 1kg)
![alt text](https://github.com/yymmaa0000/Trajectory_Optimization_of_2-link_Robotic_Arm/blob/master/Sample%20result/arm%20and%20object%20equally%20heavy.gif)

Optimized trajectory with the object being relatively light  
(mass of arm 1 = 10kg, mass of arm 2 = 1kg, mass of object = 1kg)
![alt text](https://github.com/yymmaa0000/Trajectory_Optimization_of_2-link_Robotic_Arm/blob/master/Sample%20result/very%20heavy%20arm.gif)

Optimized trajectory with the object being relatively heavy  
(mass of arm 1 = 1kg, mass of arm 2 = 1kg, mass of object = 10kg)
![alt text](https://github.com/yymmaa0000/Trajectory_Optimization_of_2-link_Robotic_Arm/blob/master/Sample%20result/Very%20heavy%20object.gif)
