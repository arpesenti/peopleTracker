% EXPANDCANDIDATES Expand foot candidates in leg and person candidates,
% building also the leg rectified depth image (silhouette)
%
% candidates = expandCandidates(candidates, centroidsOnFloor, voxelGridPoints, voxelGridColors)
%
% Inputs:
%   candidates = 1xN initialized array of N candidate structs
%                containing the candidates foot points to be expanded
%   centroidsOnFloor = 2x(N+M) matrix containing N candidates feet centroids projected on the floor plane
%                      and M other cluster centroids (rejected for too
%                      large or too small dimensions)
%   voxelGridPoints = 3xNumPoints matrix containing all the 3D point cloud,
%                     excluding the floor.
%   voxelGridColors = 3xNumPoints matrix containing the RGB components for
%                     every point. If empty, no plot needed, and so colors
%                     are not added to candidate structs.
%
% Output:
%   candidates = 1xN array of N candidate structs containing the foot
%                expansions and the leg rectified depth image.
%
% See also GETCANDIDATELEGS