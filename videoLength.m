function [ len ] = videoLength( kinectHandle )
%VIDEOLENGTH compute the total number of frames of a recorded .oni file
%
%Input:
%   kinectHandle = kinect handle
%
%Output:
%   len = scalar indicating the number of frames

% compute MD5 digest of each frame

photo = mxNiPhoto(kinectHandle);
s = 512;
string = num2str(photo(1:s));
string(ismember(string,' ')) = [];
firstHash = mMD5(string);
hash = '';
len = 1;

while ~strcmp(firstHash, hash)
   mxNiUpdateContext(kinectHandle);
   photo = mxNiPhoto(kinectHandle);
   string = num2str(photo(1:s));
   string(ismember(string,' ')) = [];
   hash = mMD5(string);
   len = len + 1;
end

end

