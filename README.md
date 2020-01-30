# V-REP-Robotics-Simulation
Algorithms implementation for intelligent agents on the V-REP simulator.

## **Line Follower Avoiding Obstacles**

Click image to see the video.

[![Youtube video](https://github.com/rosasalberto/V-REP-Robotics-Simulation/blob/master/images/_line_follower.PNG)](https://www.youtube.com/watch?v=QesoXU_kqDU)

The agent has to follow the black line avoiding the obstacles. When the agent arrives to the red line it has to stop.
The agent reads information from the sensors, three 1x1 resolution cameras and one radial ultrasonic sensor implemented on SensorInformation class.
The Controller class specify the deterministic action depending on the sensor information readings.
The implementation was made using the remote API for Python client.

## **Maze Solver**

Click image to see the video.

[![Youtube video](https://github.com/rosasalberto/V-REP-Robotics-Simulation/blob/master/images/_maze_solver.PNG)](https://www.youtube.com/watch?v=3Ijwl4Uje7I)

The agent has to solve a maze. The right hand implementation is made. The agent follows the right wall until arriving to the end of the maze. This implementation does not solve ciclic mazes.  
The Lua implementation is available on the simulator script Pioneer_p3dx.

## **Sumo Battle**

Click image to see the video.

[![Youtube video](https://github.com/rosasalberto/V-REP-Robotics-Simulation/blob/master/images/_sumo_battle.PNG)](https://www.youtube.com/watch?v=w4VC9Quo1sM)

The agent has to fight against other agent. First robot to get out from the black line lose. 
Implemented a custom modified Braitenberg algorithm to orient the agent and go against the rival.
The Lua implementation is available on the simulator script Pioneer_p3dx.
