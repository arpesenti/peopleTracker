function [best_plane, best_count] = getGroundPlane( points, maxInclinationAngle, tol )
%GETGROUNDPLANE Takes a struct voxelGrid as obtained from createVoxelStruct and returns the parameters of
%the plane which most probably is the ground plane, following the RANSAC
%plane fitting algorithm
%
%NOTE: this function works well when the floor is a significantly 
%extended planar surface in the bottom of the image. Specifying a 
%maxInclinationAngle can improve results also when seeing only limited floor
%areas in the photo.
%
% [best_plane, PNout, best_count] = getGroundPlane( voxelGrid, maxInclinationAngle, tol )
%
%   input:      
%       points     = points as obtained from getVoxelGrid
%       
%       maxInclinationAngle = (optional) scalar number (in degrees) expressing the
%                     maximum inclination angle that the camera can be
%                     submitted to (both pitch and roll) w.r.t. an ideal position
%                     that look exactly straight and parallel to the floor plane.
%                     If not present, the algorithm assumes that the camera
%                     can be inclinated to a max of 80 degree. Restricting
%                     the maximum inclination can help the algorithm in
%                     finding the correct floor plane also in proximity of
%                     vertical walls. The lowest value is however 10 degree
%                     and the maximum 90 degree.
%       tol         = (optional) tolerance to be used to test when a point
%                     belongs to a plane. Default value is 1e-6.
%
%   output:
%       best_plane  = column vector containing the [a b c d] parameters of the
%                     plane  a*x + b*y + c*z + d = 0 which is considered to
%                     be the ground plane.
%       best_count  = number of votes received by best_plane
%       
%
% See also getPointCloud

  % collinearity threshold
  collinearityThreshold = sin(0.1*pi/180);
    
  % set maximum inclination angle of the camera, if not provided as input
  if nargin < 2
    maxInclinationAngle = 80;
  end
  
  if maxInclinationAngle > 90
    maxInclinationAngle = 90;
  end
  
  % allow always some tolerance for inclination
  if maxInclinationAngle < 10
    maxInclinationAngle = 10;
  end
  
  % tolerance for a point to belong to a plane
  if nargin < 3
    % set default tolerance
    %tol = 1e-6;
    tol = 2; % distance in mm from plane
  end

  % variables subsequently used to determine if a vector can be the normal
  % to the ground plane or not (if not they probably are a wall). They are 
  % set according to the maxInclinationAngle of the camera.
  minTolWall = deg2rad(maxInclinationAngle);
  maxTolWall = deg2rad(180 - maxInclinationAngle);
  
  % max number of tentative
  tentative_num = 30;
  
  % cut points
  points = points(:,points(2,:)<0);
  pointsHom = [points; ones(1,size(points,2))];
  
  if size(points,2) < tentative_num * 3
    best_plane = [];
    best_count = 0;
    return;
  end
  
  % if count greater than this threshold, accept plane
%   thresholdCount = round(0.8*size(XYZ,1));
  
  % initializations
  best_count = 0;
  best_plane = [];
    
  while tentative_num>0 
    
    % randomly select 3 points
    idxs = randperm(size(points,2),3);
    P1 = points(:,idxs(1)); 
    P2 = points(:,idxs(2));
    P3 = points(:,idxs(3));
    
    % plane through the 3 points
    u = P2-P1;
    u = u/sqrt(sum(u.^2));
    v = P3-P1;
    v = v/sqrt(sum(v.^2));
    planeNormal = [u(2)*v(3)-u(3)*v(2); u(3)*v(1)-u(1)*v(3); u(1)*v(2)-u(2)*v(1)];
    
    planeNorm = sqrt(sum(planeNormal.^2));
    
    if planeNorm < collinearityThreshold
      % the 3 points are not non-colinear
      % disp('the 3 points are NOT OK, skip them')
      continue; 
    end
    
    % check if could be the floor according to maxInclinationAngle
    planeToCheck = planeNormal/planeNorm;
    angleWithYaxis = acos(([0 1 0]*planeToCheck));
     if angleWithYaxis > minTolWall && angleWithYaxis < maxTolWall
       % probably its a Vertical plane
       % disp('this was not the floor plane - check max inclination angle');
       continue;
     end
     
    d = -planeToCheck'*P1; 
    plane = [planeToCheck; d]./d;
    
    count = nnz(abs(plane'*pointsHom) < sqrt(sum(plane(1:3).^2))*tol);

    if count > best_count
      best_count = count;
      best_plane = plane;
    end

%     if best_count >= thresholdCount
%       % if one plane reaches the threshold of votes, stop the algorithm in
%       % advance
%       disp('Reached the threshold')
%       
%       break
%     end
    
    tentative_num = tentative_num - 1;
       
  end
  
  if best_plane(2) < 0
     best_plane = -best_plane;
  end
  
end

