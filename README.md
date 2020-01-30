# V-REP-Robotics-Simulation
Algorithms implementation for intelligent agents on the V-REP simulator.

## **Line Follower Avoiding Obstacles**

Click the image to see the video.

[![Youtube video](https://github.com/rosasalberto/V-REP-Robotics-Simulation/blob/master/images/_line_follower.PNG)](https://www.youtube.com/watch?v=QesoXU_kqDU)

The agent has to follow the black line avoiding the obstacles. When the agent arrives at the red line it has to stop.
The agent reads information from the sensors, three 1x1 resolution cameras and one radial ultrasonic sensor implemented on SensorInformation class.
The Controller class specifies the deterministic action depending on the sensor information readings.
Implemented using the remote API for Python client.

## **Maze Solver**

Click the image to see the video.

[![Youtube video](https://github.com/rosasalberto/V-REP-Robotics-Simulation/blob/master/images/_maze_solver.PNG)](https://www.youtube.com/watch?v=3Ijwl4Uje7I)

The agent has to solve the maze. Implemented the right-hand rule, the agent follows the right wall until arriving at the end of the maze. This implementation does all mazes.  
The Lua implementation is available on the simulator script Pioneer_p3dx.

## **Sumo Battle**

Click the image to see the video.

[![Youtube video](https://github.com/rosasalberto/V-REP-Robotics-Simulation/blob/master/images/_sumo_battle.PNG)](https://www.youtube.com/watch?v=w4VC9Quo1sM)

The agent has to fight against another agent. The first robot to get out from the black line lose. 
Implemented a custom modified Braitenberg algorithm to orient the agent and go against the rival.
The Lua implementation is available on the simulator script Pioneer_p3dx.

# How to run the simulations.
    To simulate maze_solver and sumo_battle just open the environment/simulator and run the file.
    To simulate line_follower_avoiding_obstacle:
     1. Open the environment/simulator
     2. Install NumPy dependency
     3. Make sure you have the correct dll for your OS in the vrep_api folder.
     4. Run main.py


