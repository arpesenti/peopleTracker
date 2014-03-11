%MINDIST compute the minimum distance between 2 sets of 3D points, as the
%minimum distance amongst all the possible distances between a point in the
%first set and a point in the second set
%
%[minDistVal] = minDist( points1, points2, centroid1, centroid2, threshold )
%
%Input:
%   points1 = 3xN matrix containing N 3D points
%   points2 = 3xM matrix containing M 3D points
%   centroid1 = 3x1 matrix containing the centroid of points1
%   centroid2 = 3x1 matrix containing the centroid of points2
%   threshold = (optional) if a threshold is provided (with the same unit 
%               measurement of points1 and points2), the computation will
%               stop as soon as two points have been found to have a
%               distance below the threshold. The default value used for
%               the threshold is 0.
%
%Output:
%   minDistVal = minimum distance between a point in points1 and a point in
%                points2. if a threshold is provided in input, this value
%                will be below the threshold but it's not guaranteed to be
%                the actual minimum amongst all the possible distances

