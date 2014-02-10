% EXTRACTFEATURESHOG Extract the HOG features from the legs rectified depth
% images, found in the canidate struct. HOG parameters are defined in the
% source file.
%
% features = extractFeaturesHOG(candidates)
%
% Input:
%   candidates = 1xN vector of candidate structs as obtained by getCandidateLegs
%
% Output:
%   features = MxN matrix containg M-dimensional HOG feature for each candidate
%
% See also GETCANDIDATELEGS