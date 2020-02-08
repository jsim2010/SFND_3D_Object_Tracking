# Track an Object in 3D Space

## FP.0 Final Report

You're reading it.

## FP.1 Match 3D Objects

I implemented `matchBoundingBoxes()` by finding the matches that were in a bounding box both in the previous and current frames. These were tracked by counting each instance where a previous box was matched with a current box. Then, each previous bounding box was mapped to the current bounding box with which it matched the most times.

## FP.2 Compute Lidar-based TTC

I implemented `computeTTCLidar()` by first storing all lidar points from the previous and current frames in sorted sets. Thus the minimum points would be the first in the sets. To prevent outliers from messing up the calculations, I searched through each set until I found a point that was within a tolerance value of the next point.

## FP.3 Associate Keypoint Correspondences with Bounding Boxes

I implemented `clusterKptMatchesWithROI` by finding all matches that were contained in the given bounding box. Then I calculated the mean of the matches distances and only kept matches whose distance was within a tolerance of the mean.

## FP.4 Compute Camera-based TTC

In `computeTTCCamera()`, I computed the distance ratios between all matched keypoints. To deal with outlier data, I used the median of all the distance ratios for the TTC calculation.

## FP.5 Performance Evaluation 1

1 - 4th image Several lidar points that are slightly off but equal to each other may have fooled my outlier detection
2 - 8th image Image before seems to have been too close, throwing off this calculation
