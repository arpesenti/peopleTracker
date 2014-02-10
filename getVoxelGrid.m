%GETVOXELGRID starting from the depth image, compute the 3D points and 
%downsample them using a voxel grid approach (return also the relative 
%colors if rgb is not empty)
%   
%[ points,colors ] = getVoxelGrid( depth, rgb )
%
%Input:
%   depth = 640x480 or 320x240 depth image of uint16
%   rgb = 640x480x3 or 320x240x3 matrix of uint8 containing the rgb colors. 
%         It can be an empty matrix if the user doesn't care the colors. 
%         If it's not empty, its resolution must match the depth resolution. 
%
%Output:
%   points = 3xN matrix of double containing the downsampled 3D points
%   colors = 3xN matrix containing the colors of the corrisponding points
%            or an empty matrix if the input rgb was empty 

