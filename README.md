# Eleanor-Rig-B-ball
Our ECE 470: Intro to Robotics group project. Ealeanor Rig-B-Ball is a robot that will return a basketball to the shooter when they are practicing by themselves. (It's for all the lonely people)

## Parts

#### Robot
For the robot, we use the KUKA YouBot, which is a mobile robot with a robotic arm. The robot will be able to move around the basketball court to locate and pick up the ball, use the arm to retrieve the ball and then use additional functionality to return the ball to the user (details of additional functionality are still under consideration). 

#### Sensors
For deteting the ball, we have made it such that the robot always knows the position of the ball. Physically, we can imagine it to be a tracker placed on the ball, or a special ball that comes with an inbuilt position tracker.

#### Actuators
The robot will have a large suction gripper that will assist in picking up the basketball. The robot will also have a basket to contain the ball so that the suction gripper does not always have to be turned on when the ball is in possession.

## Task Breakdown

#### Retrieving the Ball
The robot will use the information about the position of hte ball and its own position in the world frame to calculate its movement. Once it is near the ball, it will extend its arm to pick up the ball. We plan to replace the end effector a tthe arm with a suction gripper to help retrieve the ball easier.

#### Returning the ball
The robot will then carry the ball and using the same method that it did to move to the ball, move to the player and release the ball for th eplayer to grab and continue playing.
