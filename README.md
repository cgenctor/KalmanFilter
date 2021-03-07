# KalmanFilter
The Kalman Filter equations are implemented to track the robot <br>
in an image sequence and estimates the robots position, when its hidden behind a <br>
wall. To get measurements of the position of the robot, a simple pattern <br>
matching approach is used, which is based on cross-correlation. <br>
<br>
Tracking the Robot according to the correlation between images and the pattern: 
![](ResultData/ResultAnimated.gif)
<br>
Initial state estimation = [1470;700;0;0]             | Initial state estimation = [0;0;0;0]
:-------------------------:|:-------------------------:
![](ResultData/KalmanFilterInitialState_1470_700.png)  |  ![](ResultData/KalmanFilterInitialState_0_0.png)
