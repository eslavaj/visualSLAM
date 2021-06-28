# visualSLAM
Contains C++ Opencv implementation of Visual SLAM (IN PROGRESS)

Please note that this is a IN PROGRESS work.

Currently only Visual odometry is implemented:
- I used ORB to extract keypoints.
- Keypoint matching between consecutives frames.
- I used projective geometry constraints to refine matches (Fundamental matrix and homography).
- Once matches are relatively OK I can use camera calibration parameters to find Essential matrix, camera pose and triangulate the keypoints to estimate their 3D position up to scale. During this process a final oulier rejection is performed to keep only few but sure points as landmarks.


TODO: 
- Local optimization with g2o or other optimization framework.
- Define a good data structure and representation for mapping (for embedded platforms)
- Global optimization
- Use all this for path planning


### These are preliminary results of odometry using Kitty dataset running on NVidia Jetson TX2 board:


![Visual odometry demo with Kitty dataset](media/camposedemo.gif)



## Commit legend:
feat: a change to implement a new feature

chore: a change does not impact functionnality like clean up the code

doc: a change to add documentation

fix: a change correct a bug, generally linked to an issue 




