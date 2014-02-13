function [rotatedPoints, M, Minv] = rotatePointCloud(points, floorParameters)
%ROTATEPOINTCLOUD transforms the 3D coordinates of the point cloud from the
%reference system of the camera to the following reference system:
%- origin of the system into the orthogonal projection of the camera system
%origin onto the floor plane identified by floorParameters;
%- z axis going "upwards" orthogonally to the floor plane
%- y axis lying on the floor plane and going forward along the direction 
%resulting by projecting the camera z-axis onto the floor plane
%- x axis determined to be a right-handed system
%
%
%NOTE: 
% IMPORTANT PRECONDITION: the camera reference system (w.r.t. which the input
% points coordinates are expressed) must satisfy the following constraint: the
% y-axis of the camera reference system must intersect the floor plane 
% specified by floorParameters at a certain point in its NEGATIVE
% SEMI-AXIS (sensor is not upside down).
%
%
% [rotatedPoints, M, Minv] = rotatePointCloud(points, floorParameters)
%
%   input:
%     points         = 3xN matrix containing the 3D point cloud, as
%                      obtained by getVoxelGrid
%     floorParameters   = vector containing the a, b, c, d parameters defining 
%                         the floor plane: a*x + b*y + c*z + d = 0.
%
%   output:
%     rotatedPoints  = 3xN transformed points. Coordinates
%                      specified in the new reference system described above. 
%     M = 4x4 direct transformation matrix
%     Minv = 4x4 inverse transformation matrix
%
% See also GETVOXELGRID


    % ensure to have the normal vector of the floor pointing "upwards",
    % which corresponds to have positive value of y-component in the reference
    % system of the camera. This assume that the sensor is not used upside
    % down.
    if floorParameters(2) < 0
        floor = floorParameters*(-1);
    else 
        floor = floorParameters;
    end
    
    xAxis = [1;0;0];
    zAxis = [0;0;1];
    
    floorVersor = reshape(floor(1:3)/norm(floor(1:3)), 3, 1);
    tol = 1e-6;
    if sqrt(sum((floorVersor - zAxis).^2))<tol
        % Point cloud already in the correct position
        rotatedPoints = points;
        return;
    end
    
    % obtain the axis (see angle next) of rotation of camera zAxis w.r.t. the floorVersor
    axisOfRotation = [floorVersor(2).*zAxis(3)-floorVersor(3).*zAxis(2); 
                      floorVersor(3).*zAxis(1)-floorVersor(1).*zAxis(3);
                      floorVersor(1).*zAxis(2)-floorVersor(2).*zAxis(1)]; % cross product of floorVersor with zAxis
    axisOfRotation = axisOfRotation/norm(axisOfRotation);
    
    % 1) determine rotation of the x-axis to correct eventual roll of camera
   
    roll = acos(dot(axisOfRotation, xAxis)/(norm(axisOfRotation)*norm(xAxis)));
    
    % determine the direction of rotation of the x-axis    
    if axisOfRotation(2) > 0
      angleRotX = -roll;
    else
      angleRotX = +roll;
    end
    
    % rotation around z-axis to align x-axis to axisOfRotation (that is
    % parallel to the floor and orthogonal to z-axis)
    RotOfX = rotationMatrixFromAxisAndAngle(zAxis, angleRotX);
            
    % 2) rotate camera plane xy of PI
    RotOfXY = rotationMatrixFromAxisAndAngle(zAxis, -pi);

    % 3) rotate around new x-axis to make zAxis pointing to plane versor direction
    
    % angle between floor Versor and zAxis
    angle = acos(dot(floorVersor, zAxis)/(norm(floorVersor)*norm(zAxis)));
    
    % obtain matrix expressing the rotation of z axis
    R = rotationMatrixFromAxisAndAngle(xAxis, -angle);
    
    % final rotation matrix
    rot = R * RotOfXY * RotOfX;
   
    % obtain translation
    dist = abs(floor(4))/norm(floor(1:3));
    T = [0; 0; dist];
    
    % apply transformation 
    rotatedPoints = rot*points + repmat(T,1,size(points,2));
    M = [rot T; 0 0 0 1]; % direct transformation
    invRotOfX = rotationMatrixFromAxisAndAngle(zAxis, -angleRotX);
    invRotOfXY = rotationMatrixFromAxisAndAngle(zAxis, pi);
    invR = rotationMatrixFromAxisAndAngle(xAxis, angle);
    Minv = [invRotOfX*invRotOfXY*invR zeros(3,1); 0 0 0 1] * [eye(3) -T; 0 0 0 1];  % inverse transformation

end

function [matrix] = rotationMatrixFromAxisAndAngle(axis, angle)
%ROTATIONMATRIXFROMAXISANDANGLE returns the matrix which performs a rotation
%around a generic axis passing through the origin of a desired angle.
%
%   [matrix] = rotationMatrixFromAxisAndAngle(axis, angle)
%
%
%   input:
%          axis     =  a vector in 3D space indicating the direction of the  
%                      axis of rotation
%          angle    =  the angle of the rotation
%
%   output:
%          matrix   =  3x3 matrix performing a rotation around axis 'axis' of 
%                      an angle 'angle' 


    unitAxis = axis./norm(axis);
    x = unitAxis(1);
    y = unitAxis(2);
    z = unitAxis(3);
    
    cosAngle = cos(angle);
    sinAngle = sin(angle);
    
    matrix = [cosAngle+x*x*(1-cosAngle)    x*y*(1-cosAngle)-z*sinAngle   x*z*(1-cosAngle)+y*sinAngle;
              y*x*(1-cosAngle)+z*sinAngle  cosAngle+y*y*(1-cosAngle)     y*z*(1-cosAngle)-x*sinAngle;
              z*x*(1-cosAngle)-y*sinAngle  z*y*(1-cosAngle)+x*sinAngle   cosAngle+z*z*(1-cosAngle)];
            
end