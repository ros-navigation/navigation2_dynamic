# kf_hungarian_tracker

This package implememts a multi-object tracker with Kalman filter and Hungarian algorithm. 
Hungarian algorithm is used to associate detection and previously known objects. 
For each object, a Kalman filter is maintained to track the position and velocity. 

### Hungarian algorithm

[Hungarian algorithm](https://en.wikipedia.org/wiki/Hungarian_algorithm) is a combinatorial optimization algorithm that solves the assignment problem in polynomial time. 
Here we use an implementation from `scipy.optimize.linear_sum_assignment`. The default cost function is Euclidean distance between two object's center. 
More cost functions like IoU of bounding boxes could be used.

### Parameters

Parameters can be set in `config/kf_hungarian.yaml`.

| parameters       | Meaning        | Default |
| ---------------- | ------------- | ------- |
| global_frame     | transform from pointcloud frame to global frame,<br>None means current frame is global  | camera_link   |
| death_threshold  | maximum missing frames before deleting an obstacle  | 3 |
| top_down | whether project 3D points on ground plane | False |
| measurementNoiseCov | measurement noise for Kalman filter | [1., 1., 1.] |
| errorCovPost | initial posteriori error estimate covariance matrix | [1., 1., 1., 10., 10., 10.] |
| a_noise | model process noise covariance with estimated acceleration | [2., 2., 0.5] |
| vel_filter | minimum and maximum velocity to filter obstacles | [0.1, 2.0] |
| height_filter | minimum and maximum height (z) to filter obstacles | [-2.0, 2.0] | 
| cost_filter | filter Hungarian assignment with cost greater than threshold | 1.0 |
