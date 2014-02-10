function [img] = plotPC(points, colors, R, overlayPoints, overlayColors, alphaOverlay, alphaBackground)
% PLOTPC Generate an image of the point cloud, optionally with a colored overlay
%
% Inputs:
%   points = 3xN matrix containing the point cloud.
%   colors = 3xN RGB component for each point.
%   R = 4x4 inverse transformation matrix as obtained by rotatePointCloud (Minv)
%   overlayPoints = (optional) 3xM matrix containing points to be added to 
%                   the point cloud plot.
%   overlayColors = (optional) 3xM RGB component for each added point. If
%                   not passed, the color is chosen in function of the distance 
%                   from the sensor.
%   alphaOverlay = (optional) scalar indicating the opacity of overlay points
%   alphaBackground = (optional) scalar indication the opacity of original points
%
% Output:
%   img = 480x640x3 matrix containg a representation of the point cloud.
%         Use imshow to see the obtained image
%
% See also GETVOXELGRID, ROTATEPOINTCLOUD, IMSHOW

if isempty(points)
    img = [];
    return;
end
if nargin < 7
    alphaBackground = 1;
end

% camera extrinsic parameters
viewPointRotation = eye(3);
viewPointTraslation = [0;0;0];

if isa(colors,'uint8')
    colors = cast(colors,'double')./255;
end

% output image resolution
sizex = 640;
sizey = 480;

% xtion
fieldOfViewH = 58/180*pi; %rad
fieldOfViewV = 45/180*pi; %rad

% kinect 360
% fieldOfViewH = 57/180*pi; %rad
% fieldOfViewV = 43/180*pi; %rad
    
yMax = 4000;
yMin = 0;

if size(colors,1) == 1
    % use colors as indexes on color map
    numclass = 256;
    cmap = hsv(numclass);
    minColors = min(colors,[],2);
    maxColors = max(colors,[],2);
    ii = floor( (colors - min(yMin,minColors) ) * (numclass-1) / (max(yMax,maxColors) - min(yMin,minColors)) );
    ii = ii + 1;
    colors = cmap(ii,:);
end

fx = sizex/2/tan(fieldOfViewH/2)*0.88;
fy = sizey/2/tan(fieldOfViewV/2)*0.86;

viewPoint = [-viewPointTraslation(1); -viewPointTraslation(3); viewPointTraslation(2)];

K = [fx 0 sizex/2; 0 fy sizey/2; 0 0 1]; % camera intrinsic parameters
P = K*[viewPointRotation*R(1:3,1:3) viewPointTraslation+R(1:3,4)]; % camera extrinsic parameters

img = zeros(sizey, sizex, 3);

% draw points
uvw = (P*[points; ones(1,size(points,2))]);
u = round(sizex-uvw(1,:)./uvw(3,:)+1);
idxToDeleteU = find(u<=0 | u>sizex);
v = round(sizey-uvw(2,:)./uvw(3,:)+1);
idxToDeleteV = find(v<=0 | v>sizey);
idxToDelete = union(idxToDeleteU, idxToDeleteV);
u(idxToDelete) = [];
v(idxToDelete) = [];
colors(:,idxToDelete) = [];
points(:,idxToDelete) = [];
idx = sizey*(u-1) + v;
dists = pdist2(viewPoint', points');
[~,idxSort] = sort(dists,2,'descend');
idx = idx(idxSort);
colors = colors(:,idxSort);
img(idx) = alphaBackground.*colors(1,:);
img(idx+sizex*sizey) = alphaBackground.*colors(2,:);
img(idx+2*sizex*sizey) = alphaBackground.*colors(3,:);

% draw overlay
if nargin >= 4 && ~isempty(overlayPoints)
    if isa(overlayColors,'uint8')
        overlayColors = cast(overlayColors,'double')./255;
    end
    uvw = (P*[overlayPoints; ones(1,size(overlayPoints,2))]);
    u = round(sizex-uvw(1,:)./uvw(3,:)+1);
    idxToDeleteU = find(u<=0 | u>sizex);
    v = round(sizey-uvw(2,:)./uvw(3,:)+1);
    idxToDeleteV = find(v<=0 | v>sizey);
    idxToDelete = union(idxToDeleteU, idxToDeleteV);
    u(idxToDelete) = [];
    v(idxToDelete) = [];
    overlayColors(:,idxToDelete) = [];
    idx = sizey*(u-1) + v;
    img(idx) = img(idx)+ alphaOverlay.*overlayColors(1,:);
    img(idx+sizex*sizey) = img(idx+sizex*sizey) + alphaOverlay.*overlayColors(2,:);
    img(idx+2*sizex*sizey) = img(idx+2*sizex*sizey) + alphaOverlay.*overlayColors(3,:);
end

% saturate
img(img > 1) = 1;
img(img < 0) = 0;



