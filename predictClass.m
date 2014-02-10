function [ probEstimates ] = predictClass( svm, candidates )
%PREDICTCLASS return the probability of each candidate to be a leg
%
%Input:
%   svm = trained svm as obtained by libsvmtrain
%   candidates = array of N candidate struct.
%
%Output:
%   probEstimates = Nx1 matrix containing the probability of each candidate 
%                   to be a leg
%
%See also libsvmtrain and libsvmpredictfast (in libsvm folder)

if isempty(candidates)
    probEstimates = [];
    return;
end

inputSamples = extractFeaturesHOG(candidates);

[prob, class] = libsvmpredictfast(inputSamples, svm);

probEstimates = prob(:,class==1);

end

