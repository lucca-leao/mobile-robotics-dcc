# mobile-robotics-dcc
A bunch of mobile robotics algorithms. Simulations are done on CoppeliaSim using the Python API.

## Reference frames and Transformations
Retrieve and plot the relative transformation matrices between different objects in the scene. Also plots a robot-base-mounted laser scanner data with relation to the world frame.

![image](https://user-images.githubusercontent.com/32180331/230533610-01977ebd-0786-4edc-93b6-b900342ed1a3.png)
![image](https://user-images.githubusercontent.com/32180331/230533658-1e1b5253-122f-498f-b102-c5d014c8bd5e.png)
![image](https://user-images.githubusercontent.com/32180331/230533699-bca9836c-5195-4c92-857c-1ae6dcc1adf7.png)

## Navigation and Planning
The navigation and planning script contains algorithms (RRT, Grid Roadmap, Potential Fields) able to move a robot toward a goal while avoidind obstacles in the environment.

### RRT
![image](https://user-images.githubusercontent.com/32180331/230535490-fd686db4-e716-4817-9f52-656722dc2f8f.png)


### Grid Roadmap
![image](https://user-images.githubusercontent.com/32180331/230535209-a22e3ff6-1ed1-4091-9205-7108844de2f2.png)



### Potential Fields
![image](https://user-images.githubusercontent.com/32180331/230535037-ecc56fd3-b7a5-4ab0-a1b0-33de0e13fd86.png)

## Occupancy Grid Mapping
Builds a 2D Occupancy Grid map from laser data. The map is updated in real time while the robot is moving around the environment.

### Example maps
![image](https://user-images.githubusercontent.com/32180331/230536260-71cfd8a9-4485-45c4-a106-51402dcac6f1.png)
![image](https://user-images.githubusercontent.com/32180331/230536280-6af957e0-8336-43bb-b029-7e00128b6e53.png)

## Reactive RRT
A hybrid navigation algorithm that combines the path planning of RRT and reactive obstacle avoidance of Potential Fields, allowing the robot to move safely in dynamic environments.

![image](https://user-images.githubusercontent.com/32180331/230537421-b56b44c2-9284-4b9f-87b4-fc346bf9c3f3.png)

## Lectures
If you would like to an in-depth understanding of each algorithm, check out my video explanations (they're in Portuguese):
- [Navigation and Planning](https://youtu.be/AeXQfEIXQcg)
- [Occupancy Grid Mapping](https://youtu.be/uUqiVMOrajU)
- [Reactive RRT](https://youtu.be/gyzC2T0Ec0A)
