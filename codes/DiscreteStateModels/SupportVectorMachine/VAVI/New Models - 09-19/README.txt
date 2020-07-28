% The Fine Gaussaian SVM models perform better in 5-fold cross validation compared to thecubic SVM model when the number of features reduce!
% Uses the data with high deceleration instances.

% Features - Instantaneous gap when gap is first available, average pedestrian speed for last one second,  distance to crosswalk, distance to curb, gaze ratio in last one second, cumulative wait time

Cubic SVM performance:
1) All Six  - 85%
2) Five, No gaze - 86
3) Four, No Gaze, No Curb dist - 76
4) Three, No Speed also - 67
5) Only Gap - 39

Fine Gaussian SVM performance
1)- 4) ~(80-84%)
5) 76%