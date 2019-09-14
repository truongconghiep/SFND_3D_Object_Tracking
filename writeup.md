**SFND 3D OBJECT TRACKING**


Student: Truong Cong Hiep

------------

### 1. Matching 3D objects

Matching 3D object is implemented in "matchBoundingBoxes" method, which matches bounding boxes of current frame to bounding boxes in the previous frame. Brute force method is applied to match bounding boxes. For each bounding box in the current frame the function goes through all bounding boxes in the previous frame and count number of matched keypoints. The bounding boxes paar which have most matched keypoints in considered as a bounding boxes match. [Implementation code](https://github.com/truongconghiep/SFND_3D_Object_Tracking/blob/00685101f645d11fc5a1dabfe82df9c56789ef1e/src/camFusion_Student.cpp#L148)

### 2. Compute lidar TTC

In this step TTC is calculated based on lidar signal according to following formula:

TTC = minCurr * dT /(meanXPrev - meanXCurr)

where: minCurr is distance from the vehicle to the nearest lidar point in the current frame
       dT is time difference between 2 frames
       meanXPrev is mean value of distances from lidar points in previous frame to the vehicle
       meanXCurr is mean value of distances from lidar points in current frame to the vehicle
In this calculation mean values are calculated to eliminate the the influence of faulty lidar points. The minCurr value is also checked if it is plausible (not too far from the mean distance). 
[Implementation code](https://github.com/truongconghiep/SFND_3D_Object_Tracking/blob/00685101f645d11fc5a1dabfe82df9c56789ef1e/src/camFusion_Student.cpp#L200)

### 3. Associate keypoint correspondences with bounding boxes

In this step keypoint matches will be associated to matched bounding boxes paars. If both keypoints of a match are falling into region of interest in current and previous frame then the keypoint paar will be added to the bounding boxes match.
[Implementation code](https://github.com/truongconghiep/SFND_3D_Object_Tracking/blob/00685101f645d11fc5a1dabfe82df9c56789ef1e/src/camFusion_Student.cpp#L134)


### 4. Compute camera TTC

In this step TTC is calculated based on camera data with following formula:

TTC = -dT / (1 - medDistRatio)
where: dT is time difference between 2 frames
       medDistRatio is mean of ratios of distances between 2 keypoints on current and on previous frame.
[Implementation code](https://github.com/truongconghiep/SFND_3D_Object_Tracking/blob/00685101f645d11fc5a1dabfe82df9c56789ef1e/src/camFusion_Student.cpp#L148)

### 5. Lidar TTC evaluation

No unreasonable lidar ttc was detected. All values are in range from 8.4 to 16.1. See measurement data [here](https://github.com/truongconghiep/SFND_3D_Object_Tracking/blob/master/build/InfoLog.csv)

### 6. Accuracy evaluation
