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

Measurement of different detector/descriptor combinations is given below:

| Detector   | Descriptor | Frame | TTC lidar | TTC camera | TTC diff |
|------------|------------|-------|-----------|------------|----------|
| HARRIS     |    ORB     |   6   |  13.4805  |   13.6217  | -0.141213|
|HARRIS|	BRISK|	11|	11.6056|	11.7414|	-0.135786|
|HARRIS|	BRIEF|	11|	11.6056|	11.7414|	-0.135786|
|HARRIS|	FREAK|	11|	11.6056|	11.7414|	-0.135786|
|FAST|	SIFT|	1|	12.1403|	12.2747|	-0.134334|
|SHITOMASI|	SIFT|	9|	11.8654|	11.9899|	-0.124507|
|FAST|	BRIEF|	6|	13.4805|	13.5942|	-0.113665|
|SIFT|	BRIEF|	12|	9.77463|	9.87061|	-0.0959779|
|HARRIS|	FREAK|	7|	13.5199|	13.6036|	-0.0837262|
|ORB|	FREAK|	1|	12.1403|	12.2074|	-0.0670974|
|AKAZE|	ORB|	10|	11.7072|	11.7537|	-0.0465839|
|**SHITOMASI**|	**FREAK**|	17|	10.9079|	10.9094|	-0.00146737|**
|**AKAZE**|	**FREAK**|	1|	12.1403|	12.1239|	0.016497|
|**AKAZE**|	**BRIEF**|	10|	11.7072|	11.6673|	0.0398412|
|SIFT|	SIFT|	13|	9.31567|	9.24576|	0.0699048|
|FAST|	FREAK|	1|	12.1403|	12.0538|	0.0865223|
|SHITOMASI|	SIFT|	17|	10.9079|	10.7884|	0.119557|
|HARRIS|	ORB|	11|	11.6056|	11.4377|	0.16796|
|HARRIS|	SIFT|	11|	11.6056|	11.4377|	0.16796|
|SHITOMASI|	BRIEF|	4|	13.7868|	13.6007|	0.186071|
|SHITOMASI|	BRISK|	2|	13.1844|	12.9876|	0.196752|

As seen above the best detector/descriptor combinations are SHITOMASI/FREAK, AKAZE/FREAK and AKAZE/BRIEF

Some combination where camera-based TTC is way off:

| Detector   | Descriptor | Frame | TTC lidar | TTC camera | TTC diff |
|------------|------------|-------|-----------|------------|----------|
|HARRIS|BRIEF|18|	158|	100|	8.43837|	       -inf|	       inf|
|HARRIS|BRISK|18|	158|	73|	8.43837|	       -inf|	       inf|
|HARRIS|BRISK|5|	241|	134|	12.4891|	       -inf|	       inf|



