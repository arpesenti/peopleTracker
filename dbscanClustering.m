%DBSCANCLUSTERING clusterize points according to a given number of
%neighbors in a specific radius distance
%
%[class, type] = dbscanClustering( points, nNeigh, radius)
%
%Input:
%   points = 3xN matrix containing 3D points to clusterize
%   nNeigh = scalar value indicating the number of neighbors of a point to
%            be inserted in a cluster
%   radius = scalar value (with same unit of measurement of points)
%            indicating the radius of the sphere centered in each point to 
%            be considered as its neighborhood
%
%Output:
%   class = Nx1 matrix indicating the id (a positive integer) of the cluster
%           to which the corresponding point in points has been assigned. 
%           If the class of a point is -1 it means that it has not been 
%           inserted in any cluster (so it's an outlier point)
%   type = Nx1 matrix indicating the type of the corresponding point in
%          points. The type of a point can be -1 (outlier), 1 (core point) 
%          or 0 (border point)