# Eleanor-Rig-B-ball
Our ECE 470: Intro to Robotics group project. Ealeanor Rig-B-Ball is a robot that will return a basketball to the shooter when they are practicing by themselves. (It's for all the lonely people)

## Parts

#### Robot
Our plan at the moment is to use the KUKA YouBot, which is a mobile robot with a robotic arm. The robot will be able to move around the basketball court to locate and pick up the ball, use the arm to retrieve the ball and then use additional functionality to return the ball to the user (details of additional functionality are still under consideration). 

#### Sensors
It will also have a proximity sensor attached to it to assisit the robot in locating and retrieving the ball. Additionally it will also feature another type of sensor, the 3D laser scanner, that will be used to locate the human player and return the ball to the player.

#### Actuators
The robot will have a large suction gripper that will assist in picking up the basketball. The robot will also have a basket to contain the ball so that the suction gripper does not always have to be turned on when the ball is in possession.

## Task Breakdown

#### Retrieving the Ball
The robot will use multiple sensors (3D laser scanner for longer distances and proximity sensors for shorter distances) in conjunction to track the ball. Once the ball is ready to be retrieved, the robot will move itself towards the ball and use the suction gripper to pick up the ball.

#### Returning the ball
The robot will then either use the arm to throw the ball towards the player, or place the ball in a large basket and carry it back to the player. (Design decision pending)
 - Throwing it to the player will provide a better user experience, but will be much trickier to realize effectively.
 - Carrying it to the player on the other hand, seems to be much more feasible but would probably lead to longer waiting time in between throws.
