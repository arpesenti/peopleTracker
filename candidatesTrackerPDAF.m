function [ candidatesTrack, tracker ] = candidatesTrackerPDAF( candidates, probEstimates, tracker )
%CANDIDATESTRACKERPDAF tracking of all candidate legs 
%   
%Input:
%   candidates = array of N candidate struct
%   probEstimates = Nx1 matrix containing the probability of each candidate 
%                   to be a leg
%   tracker = tracker struct
%
%Output:
%   tracker = updated tracker struct
%   candidatesTrack = Nx1 matrix containing the index of single leg track 
%                     to which each candidate has been associated


% --- PARAMETERS ---
NUM_FRAMES_NOT_SEEN = 5; % max number of time steps a track can survive without 
                         % having received an associable measure 

distanceThresholdForStraightAssociation = 50; % mm
distanceThresholdForNewTrackCreation = 150; % mm
distanceThresholdToOtherCandidates = 100; % mm
distanceThresholdForAssociationMultiple = 100; %mm

probThresholdNewCreation = 0; % (previously 0.2 for time performances)  
% ------------------


% retrieve data from tracker
tracks = tracker.legTracks;
freeID = tracker.legFreeID;
sigmaZ = tracker.legSigmaZ;
sigmaP = tracker.legSigmaP;
sigmaAcc = tracker.legSigmaAcc;

% compute time elapsed from last update
DT = tracker.currentTimestamp - tracker.oldTimestamp;
                       
                         
candidatesTrack = zeros(length(candidates),1);

idxToKill = [];

% predict step of Kalman Filter
for i = 1:length(tracks)
  tracks(i).prediction = KFpredict(tracks(i).prediction, sigmaAcc, DT);
  tracks(i).lastSeen = tracks(i).lastSeen + 1;
  if tracks(i).lastSeen > NUM_FRAMES_NOT_SEEN
    % remove track
    idxToKill = [idxToKill i];
  end
end

tracks(idxToKill) = [];

if isempty(candidates)
  candidatesTrack = [];
  return;
end


% retrieve robot odometry for computing absolute measures
yaw = tracker.pose(3);
cyaw = cos(yaw);
syaw = sin(yaw);

tmpCentroids = cat(2,candidates.centroid); 
centroids = ([cyaw -syaw; syaw cyaw]*tmpCentroids(1:2,:) + repmat([tracker.pose(1); tracker.pose(2)].*1000, 1, size(tmpCentroids,2)))';

if ~isempty(tracks)
  predictions = cat(1,tracks.prediction);
  states = cat(2,predictions.x);
  trackCentroids = states(1:2,:)' .* 1000; % mm
  trackCentroidsRelative = ([cyaw syaw; -syaw cyaw]*(trackCentroids' - repmat([tracker.pose(1); tracker.pose(2)].*1000, 1, size(trackCentroids,1))))'; % mm
end


newTracks = [];

updatedTracks = zeros(length(tracks),1);
candidatesTracksProximity = zeros(length(candidates), length(tracks));

% directly associate a candidate to a track if candidate foot covers track's centroid and then update 
for i=1:length(candidates)
  if ~isempty(tracks)
    distanceMatrix = pdist2(trackCentroidsRelative, candidates(i).footPoints(1:2,:)');
    minDistanceMatrix = min(distanceMatrix,[],2);
    idxTracks = find(minDistanceMatrix <= distanceThresholdForStraightAssociation);
  else
    minDistanceMatrix = Inf;
    idxTracks = [];
  end  
  
  if length(idxTracks) == 1
    % 1 candidate is straight associated to 1 track
    measures = [centroids(i,:) probEstimates(i)*2-1];    
    [tracks(idxTracks).prediction, ~, ~] = PDAFupdate(tracks(idxTracks).prediction, measures, sigmaZ,sigmaP); 
    tracks(idxTracks).lastSeen = 0;
    candidatesTrack(i) = idxTracks;
    updatedTracks(idxTracks) = 1;
  elseif length(idxTracks) >= 2
    % 1 candidate covers 2 tracks - update 2 tracks
    lastSeen = cat(1,tracks(idxTracks).lastSeen);
    [~, sortIdx] = sort(lastSeen, 'ascend');
    orderedIdxTracks = idxTracks(sortIdx);
    idxTrack1 = orderedIdxTracks(1);
    idxTrack2 = orderedIdxTracks(2);    
    
    jointCentroid = (trackCentroids(idxTrack1,:) + trackCentroids(idxTrack2,:))./2;
    candidateCentroid = centroids(i,:);
    shiftVector = candidateCentroid - jointCentroid;
    
    % track 1
    measures = [trackCentroids(idxTrack1,:)+shiftVector probEstimates(i)*2-1];
    [tracks(idxTrack1).prediction, ~, ~] = PDAFupdate(tracks(idxTrack1).prediction, measures, sigmaZ,sigmaP);
    tracks(idxTrack1).lastSeen = 0;       
    % track 2
    measures = [trackCentroids(idxTrack2,:)+shiftVector probEstimates(i)*2-1];
    [tracks(idxTrack2).prediction, ~, ~] = PDAFupdate(tracks(idxTrack2).prediction, measures, sigmaZ,sigmaP);
    tracks(idxTrack2).lastSeen = 0;
    
    candidatesTrack(i) = idxTrack1;
    updatedTracks(idxTrack1) = 1;
    updatedTracks(idxTrack2) = 1;
    
  else
    % candidate not associate to any of existing tracks
    candidatesTracksProximity(i, minDistanceMatrix < distanceThresholdForNewTrackCreation) = 1;
    
    distanceToOtherCandidates = pdist2(centroids(i,:), centroids);
    distanceToOtherCandidates(i) = Inf;
    if min(minDistanceMatrix) > distanceThresholdForNewTrackCreation && min(distanceToOtherCandidates) > distanceThresholdToOtherCandidates && probEstimates(i)>probThresholdNewCreation
      % create a new leg track
      newTrack.prediction = KFinitialize(centroids(i,:)',probEstimates(i)*2-1);
      measures = [centroids(i,:) ,probEstimates(i)*2-1];
      [newTrack.prediction, ~, ~] = PDAFupdate(newTrack.prediction, measures, sigmaZ,sigmaP);
      newTrack.lastSeen = 0;
      newTrack.id = freeID;
      freeID = freeID + 1;
      newTracks = [newTracks newTrack];  
      candidatesTrack(i) = length(tracks)+length(newTracks);
    end    
  end

end

% update the remaining tracks using PDAF approach
for j = 1:length(tracks)
  if ~updatedTracks(j)
    candidateIdx = find(candidatesTracksProximity(:,j) ~= 0);
    if length(candidateIdx) > 0
      % update using only the measures in the neighborhood
      measures = [centroids(candidateIdx,:) probEstimates(candidateIdx)*2-1];
      [tracks(j).prediction, ~, ~] = PDAFupdate(tracks(j).prediction, measures, sigmaZ,sigmaP);
      tracks(j).lastSeen = max(0, tracks(j).lastSeen - 1);  
    end
  end  
end

% try to associate to something the unassociated candidates
if ~isempty(tracks)
    for j=1:length(candidatesTrack)
        if candidatesTrack(j) == 0
            [minVal, minId] = min(pdist2(candidates(j).centroid(1:2)', trackCentroidsRelative));
            if minVal < distanceThresholdForAssociationMultiple
                candidatesTrack(j) = minId;
            end
        end
    end
end

tracks = [tracks newTracks];

tracker.legTracks = tracks;
tracker.legFreeID = freeID;
end

