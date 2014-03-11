function [ legTrackOnPeople, tracker] = peopleTrackerPDAFOnLegTracks( candidates, candidatesTrack, tracker )
%PEOPLETRACKERPDAFONLEGTRACKS People tracker over the candidate tracks obtained
% by candidatesTrackerPDAF.
%
% Inputs:
%   candidates = Nx1 vector of candidate structs as obtained by getCandidateLegs
%   candidatesTrack = Nx1 vector as obtained by candidatesTrackerPDAF. 
%                     It contains the index of single leg track to which each 
%                     candidate has been associated.
%   tracker = tracker object containing the state of the single candidate tracker
%             and of this people tracker.
%   
% Outputs:
%   legTrackOnPeople = Mx1 vector containing for each leg track the index
%                      of the person to which it as been associated, or 0 if 
%                      no association was possible
%   tracker = updated tracker object

%--- PARAMETERS ---

windowSizeSecond = 1; % avgSpeed in the last windowSizeSecond
maxFps = 30;

NUM_FRAMES_NOT_SEEN = 10; % maximum life of a not updated people track
thresholdDistToOtherCentroid = 450; % mm
thresholdDist2LegsNewPeople = 100; % mm
thresholdDist2LegsAssociation = 800; %mm
thresholdDistStraightAssociation = tracker.peopleDistThreshold;%50; %mm
thresholdDistTooClosePeople = 50; % mm
thresholdDistOverlappingCandidates = 50; %mm
probabilityThresholdForAssociation = tracker.legProbabilityThreshold;%0.5;
probabilityDeviationThresholdForAssociation = 0.15;
radiusPDAF = 400; %mm
thresholdStillPerson = 0.05^2; % m/s

%------------------

legTracks = tracker.legTracks;

% exclude from further processing leg tracks with low leg probability
if ~isempty(legTracks)
    thresholdFilterLegs = 0.2;
    predictions = cat(2,legTracks.prediction);
    states = cat(2,predictions.x);
    probabilities = (states(5,:)+1)/2;
    idxTracksToKill = find(probabilities<thresholdFilterLegs);
    idxTracksToSave = find(probabilities>=thresholdFilterLegs);
    idxCandidatesToKill = [];
    for i=1:length(candidatesTrack)
        if ismember(candidatesTrack(i), idxTracksToKill)
            idxCandidatesToKill = [idxCandidatesToKill i];
        else
            candidatesTrack(i) = candidatesTrack(i) - nnz(idxTracksToKill < candidatesTrack(i));
        end
    end
    legTracks(idxTracksToKill) = [];
    candidates(idxCandidatesToKill) = [];
    candidatesTrack(idxCandidatesToKill) = [];
end

% read parameters from tracker
peopleTracks = tracker.peopleTracks;
freeID = tracker.peopleFreeID;
sigmaZ = tracker.peopleSigmaZ;
sigmaP = tracker.peopleSigmaP;
sigmaAcc = tracker.peopleSigmaAcc;

% compute time elapsed from the previous update
DT = tracker.currentTimestamp - tracker.oldTimestamp;

legTrackOnPeople = zeros(length(legTracks),1);

% update avg speed of each track
windowLength = ceil(maxFps*windowSizeSecond);
shiftVal = min(ceil(DT*maxFps), windowLength);
for i = 1:length(peopleTracks)
    peopleTracks(i).vecSpeed(:,shiftVal+1:end) =  peopleTracks(i).vecSpeed(:,1:end-shiftVal);
    vx = peopleTracks(i).prediction.x(3);
    vy = peopleTracks(i).prediction.x(4);
    peopleTracks(i).vecSpeed(:,1:shiftVal) = repmat([vx;vy],1,shiftVal);
    peopleTracks(i).avgSpeed = mean( peopleTracks(i).vecSpeed,2);
    if sum(peopleTracks(i).avgSpeed.^2) > thresholdStillPerson
        peopleTracks(i).heading = atan2(peopleTracks(i).avgSpeed(2), peopleTracks(i).avgSpeed(1));
    end
end

idxToKill = [];
% predict step of Kalman Filter and delete people tracks not seen for NUM_FRAMES_NOT_SEEN times
for i = 1:length(peopleTracks)
  peopleTracks(i).prediction = KFpredict(peopleTracks(i).prediction, sigmaAcc, DT);
  peopleTracks(i).lastSeen = peopleTracks(i).lastSeen + 1;
  if peopleTracks(i).lastSeen > NUM_FRAMES_NOT_SEEN
    % remove track
    idxToKill = [idxToKill i];
  end
end
peopleTracks(idxToKill) = [];

% check if two people tracks are too close and delete the older one
if ~isempty(peopleTracks)
    predictions = cat(1,peopleTracks.prediction);
    states = cat(2, predictions.x); 
    peopleTracksCentroid = states(1:2,:)'.* 1000; % mm    
    peopleDistances = pdist2(peopleTracksCentroid,peopleTracksCentroid);
    peopleDistances = peopleDistances + diag(Inf.*ones(length(peopleTracks),1));
    [minVals, minTracks2ID] = min(peopleDistances,[],2);
    [minVal, track1] = min(minVals);
    while ~isempty(peopleDistances) && minVal < thresholdDistTooClosePeople
        track2 = minTracks2ID(track1);
        if peopleTracks(track1).lastSeen <= peopleTracks(track2).lastSeen
            trackToKill = track2;
        else
            trackToKill = track1;
        end
        peopleDistances(trackToKill,:) = [];
        peopleDistances(:,trackToKill) = [];
        peopleTracks(trackToKill) = [];
        peopleTracksCentroid(trackToKill,:) = [];
        
        [minVals, minTracks2ID] = min(peopleDistances,[],2);
        [minVal, track1] = min(minVals);
    end
end

if isempty(legTracks)
  % no leg tracks
  legTrackOnPeople = zeros(length(tracker.legTracks),1);
  tracker.peopleTracks = peopleTracks;
  tracker.peopleFreeID = freeID;
  return;
end

predictions = cat(1,legTracks.prediction);
states = cat(2, predictions.x);
variances = cat(2,predictions.P);
legCentroids = states(1:2,:)' .* 1000; % mm
probEstimates = states(5,:)';
normProbEstimates = (probEstimates+1)./2;
probEstimateDeviations = sqrt(variances(5,5:5:end))'./2;

availableLegTracks = 1:length(legTracks);

% compute feasible associatons between leg tracks
if length(legTracks) > 1
    possibleLegsAssociation = nchoosek(1:length(legTracks),2);
    
    % remove too far associated leg Tracks
    idxToKill = find(sum(    (legCentroids(possibleLegsAssociation(:,1),:)-legCentroids(possibleLegsAssociation(:,2),:)).^2 ,2)  > thresholdDist2LegsAssociation^2 );
    possibleLegsAssociation(idxToKill,:) = [];
    
    % remove more legs association containing already straight associated legs
    visitedAssociations = zeros(size(possibleLegsAssociation,1),1);
    while ~isempty(visitedAssociations) && nnz(~visitedAssociations) > 0
        i = find(visitedAssociations==0,1,'first');
        visitedAssociations(i) = 1;
        legTrack1 = possibleLegsAssociation(i,1);
        legTrack2 = possibleLegsAssociation(i,2);
        idCandidate1 = find(candidatesTrack == legTrack1,1,'first');
        idCandidate2 = find(candidatesTrack == legTrack2,1,'first');
        if ~isempty(idCandidate1) && ~isempty(idCandidate2)
            mindist = minDist(candidates(idCandidate1).allPoints, candidates(idCandidate2).allPoints, candidates(idCandidate1).centroid, candidates(idCandidate2).centroid, thresholdDistOverlappingCandidates);
            if mindist <= thresholdDistOverlappingCandidates
                % straight association of two legs to a person
                idxToKill = find(possibleLegsAssociation(:,1)==legTrack1 | possibleLegsAssociation(:,1)==legTrack2 | possibleLegsAssociation(:,2)==legTrack1 | possibleLegsAssociation(:,2)==legTrack2);
                idxToKill = setdiff(idxToKill,i);
                possibleLegsAssociation(idxToKill,:) = [];
                visitedAssociations(idxToKill) = [];
            end
        end
    end
    
    allPossiblePeopleCentroids = (legCentroids(possibleLegsAssociation(:,1),:) + legCentroids(possibleLegsAssociation(:,2),:))./2;
    allPossiblePeopleProbEstimates = (probEstimates(possibleLegsAssociation(:,1),:) + probEstimates(possibleLegsAssociation(:,2),:))./2;
    
else
    possibleLegsAssociation = [];
    allPossiblePeopleCentroids = [];
    allPossiblePeopleProbEstimates = [];
end

if isempty(peopleTracks)
    peopleTracksCentroid = [];
end
updatedPeopleTrack = zeros(length(peopleTracks),1);
if ~isempty(peopleTracks) && ~isempty(possibleLegsAssociation)
    distanceCentroidToCentroid = pdist2(allPossiblePeopleCentroids, peopleTracksCentroid);
    [minVals, minPeopleTrackIDs] = min(distanceCentroidToCentroid,[],2);
    [minVal, minLegsAssociationID] = min(minVals);
    minPeopleTrackID = minPeopleTrackIDs(minLegsAssociationID);
else
    distanceCentroidToCentroid = [];
    minVal = Inf;
end

% update straight associated people tracks
while ~isempty(distanceCentroidToCentroid) && minVal < thresholdDistStraightAssociation
   
    if updatedPeopleTrack(minPeopleTrackID) == 1
        % people track already updated
        associationsToKill = minLegsAssociationID;
    else
        % people track straight associated
        legTrack1 = possibleLegsAssociation(minLegsAssociationID,1);
        legTrack2 = possibleLegsAssociation(minLegsAssociationID,2);
        
        availableLegTracks([legTrack1,legTrack2]) = 0;
        associationsToKill = find(possibleLegsAssociation(:,1)==legTrack1 | possibleLegsAssociation(:,1)==legTrack2 | possibleLegsAssociation(:,2)==legTrack1 | possibleLegsAssociation(:,2)==legTrack2);
    end
    
    % remove used leg tracks
    possibleLegsAssociation(associationsToKill,:) = [];
    distanceCentroidToCentroid(associationsToKill,:) = [];
    allPossiblePeopleCentroids(associationsToKill,:) = [];
    allPossiblePeopleProbEstimates(associationsToKill,:) = [];
        
    if updatedPeopleTrack(minPeopleTrackID) == 1
        % people track already updated
        [minVals, minPeopleTrackIDs] = min(distanceCentroidToCentroid,[],2);
        [minVal, minLegsAssociationID] = min(minVals);
        minPeopleTrackID = minPeopleTrackIDs(minLegsAssociationID);
        continue;
    end
    
    % straight update
    centroid = (legCentroids(legTrack1,:) + legCentroids(legTrack2,:))./2;
    legProb = (probEstimates(legTrack1) + probEstimates(legTrack2))./2;
    measures = [centroid legProb];
    [peopleTracks(minPeopleTrackID).prediction, inGate, createNewTrack] = PDAFupdate(peopleTracks(minPeopleTrackID).prediction, measures, sigmaZ,sigmaP); 
    if inGate
      peopleTracks(minPeopleTrackID).lastSeen = 0;
    end
    updatedPeopleTrack(minPeopleTrackID) = 1;
    
    % association leg-people
    legTrackOnPeople(legTrack1) = minPeopleTrackID;
    legTrackOnPeople(legTrack2) = minPeopleTrackID;
    
    [minVals, minPeopleTrackIDs] = min(distanceCentroidToCentroid,[],2);
    [minVal, minLegsAssociationID] = min(minVals);
    minPeopleTrackID = minPeopleTrackIDs(minLegsAssociationID);
end

%remove associations with low leg probability
idxLegTracksToKill = find(normProbEstimates<probabilityThresholdForAssociation); %| probEstimateDeviations>probabilityDeviationThresholdForAssociation);
availableLegTracks(idxLegTracksToKill) = 0;

if ~isempty(possibleLegsAssociation)
    idxToKill = find(ismember(possibleLegsAssociation(:,1),idxLegTracksToKill) | ismember(possibleLegsAssociation(:,2),idxLegTracksToKill));
    possibleLegsAssociation(idxToKill,:) = [];
    allPossiblePeopleCentroids(idxToKill,:) = [];
    allPossiblePeopleProbEstimates(idxToKill,:) = [];
end

% update all remaining people Tracks, for which a precise association has
% not been found, using also single leg tracks
numRemainingAssociations = size(possibleLegsAssociation,1);

% retrieve robot odometry for computing absolute measures
yaw = tracker.pose(3);
cyaw = cos(yaw);
syaw = sin(yaw);

measures = [allPossiblePeopleCentroids, allPossiblePeopleProbEstimates];
auxPossibileAssociation = possibleLegsAssociation;
for i=1:length(availableLegTracks)
  if availableLegTracks(i) ~= 0
    idCandidate = find(candidatesTrack == i,1,'first');
    if isempty(idCandidate)
        measures = [measures; legCentroids(i,:), probEstimates(i)];
    else
        measure = ([cyaw -syaw; syaw cyaw]*mean(candidates(idCandidate).legPoints(1:2,:),2) + [tracker.pose(1); tracker.pose(2)].*1000)';
        measures = [measures; measure, probEstimates(i)];
    end
    auxPossibileAssociation = [auxPossibileAssociation; availableLegTracks(i) availableLegTracks(i)];
  end
end

notUsedIdx = 1:numRemainingAssociations;

% PDAF update of not straight associated people
for i=1:length(peopleTracks)
  if ~isempty(updatedPeopleTrack) && ~updatedPeopleTrack(i) && ~isempty(measures)
    measuresIdx = find((measures(:,1) - peopleTracks(i).prediction.x(1).*1000).^2 + (measures(:,2) - peopleTracks(i).prediction.x(2).*1000).^2 < radiusPDAF^2);
    [peopleTracks(i).prediction, inGate, createNewTrack] = PDAFupdate(peopleTracks(i).prediction, measures(measuresIdx,:), sigmaZ,sigmaP, true); 
    if nnz(inGate)>0
      peopleTracks(i).lastSeen = max(0,peopleTracks(i).lastSeen - 1);
      legTrackIdx = unique(auxPossibileAssociation(measuresIdx(inGate==1),:));
      legTrackOnPeople(legTrackIdx) = i;
    end
    updatedPeopleTrack(i) = 1;
    notUsedIdx = setdiff(notUsedIdx, measuresIdx(measuresIdx <= numRemainingAssociations));
  end
end

if isempty(notUsedIdx)
    % no new track to create
    legTrackOnPeopleOld = legTrackOnPeople;
    legTrackOnPeople = zeros(length(tracker.legTracks),1);
    legTrackOnPeople(idxTracksToSave) = legTrackOnPeopleOld;
    tracker.peopleTracks = peopleTracks;
    tracker.peopleFreeID = freeID;
    return;
end

% new People track creation 
possibleNewPeopleCentroids = allPossiblePeopleCentroids(notUsedIdx,:);
possibleNewPeopleProbEstimates = allPossiblePeopleProbEstimates(notUsedIdx,:);
possibleNewPeopleLegsAssociation = possibleLegsAssociation(notUsedIdx,:);

possibleNewPeopleLegDistances = zeros(size(possibleNewPeopleLegsAssociation,1),1);
for i=1:size(possibleNewPeopleLegsAssociation,1)
    idxCandidate1 = find(possibleNewPeopleLegsAssociation(i,1) == candidatesTrack);
    idxCandidate2 = find(possibleNewPeopleLegsAssociation(i,2) == candidatesTrack);
    if length(idxCandidate1) ~=1 || length(idxCandidate2) ~=1
        possibleNewPeopleLegDistances(i) = Inf;
    else
        candidate1 = candidates(idxCandidate1);
        candidate2 = candidates(idxCandidate2);
        possibleNewPeopleLegDistances(i) = minDist(candidate1.allPoints,candidate2.allPoints,candidate1.centroid,candidate2.centroid,thresholdDist2LegsNewPeople);
    end
end

% exclude too far apart legs
idxToKill = find(possibleNewPeopleLegDistances > thresholdDist2LegsNewPeople);
possibleNewPeopleCentroids(idxToKill,:) = [];
possibleNewPeopleProbEstimates(idxToKill,:) = [];
possibleNewPeopleLegsAssociation(idxToKill,:) = [];
possibleNewPeopleLegDistances(idxToKill,:) = [];

[possibleNewPeopleLegDistances, sortIdx] = sort(possibleNewPeopleLegDistances);

possibleNewPeopleCentroids = possibleNewPeopleCentroids(sortIdx,:);
possibleNewPeopleProbEstimates = possibleNewPeopleProbEstimates(sortIdx,:);
possibleNewPeopleLegsAssociation = possibleNewPeopleLegsAssociation(sortIdx,:);

while ~isempty(possibleNewPeopleLegsAssociation)
  distancesToOtherCentroids = pdist2(possibleNewPeopleCentroids(1,:), possibleNewPeopleCentroids);
  distancesToOtherCentroids(1,1) = Inf;
  if ~isempty(peopleTracksCentroid)
    distancesToOtherPeopleTracks = pdist2(possibleNewPeopleCentroids(1,:), peopleTracksCentroid);
  else
    distancesToOtherPeopleTracks = Inf;
  end
  if min(distancesToOtherCentroids) > thresholdDistToOtherCentroid && min(distancesToOtherPeopleTracks) > thresholdDistToOtherCentroid
     legTrack1 = possibleNewPeopleLegsAssociation(1,1);
     legTrack2 = possibleNewPeopleLegsAssociation(1,2);
     
     % create new track
     newTrack.prediction = KFinitialize(possibleNewPeopleCentroids(1,:)',possibleNewPeopleProbEstimates(1));
     measures = [possibleNewPeopleCentroids(1,:) ,possibleNewPeopleProbEstimates(1)];
     newTrack.prediction = PDAFupdate( newTrack.prediction, measures, sigmaZ,sigmaP);

     newTrack.lastSeen = 0;

     newTrack.id = freeID;
     freeID = freeID + 1;
     
     
     % average speed
     newTrack.vecSpeed = zeros(2, ceil(windowSizeSecond * maxFps));
     newTrack.avgSpeed = newTrack.prediction.x(3:4);
     newTrack.heading = 3*pi/2;
     
     peopleTracks = [peopleTracks newTrack];
     
     legTrackOnPeople(legTrack1) = length(peopleTracks);
     legTrackOnPeople(legTrack2) = length(peopleTracks);
    
     idxToKill = find(possibleNewPeopleLegsAssociation(:,1)==legTrack1 | possibleNewPeopleLegsAssociation(:,1)==legTrack2 | possibleNewPeopleLegsAssociation(:,2)==legTrack1 | possibleNewPeopleLegsAssociation(:,2)==legTrack2);
  else
     idxToKill = 1;
  end
  
  possibleNewPeopleCentroids(idxToKill,:) = [];
  possibleNewPeopleProbEstimates(idxToKill,:) = [];
  possibleNewPeopleLegsAssociation(idxToKill,:) = [];
  possibleNewPeopleLegDistances(idxToKill,:) = [];
end

legTrackOnPeopleOld = legTrackOnPeople;
legTrackOnPeople = zeros(length(tracker.legTracks),1);
legTrackOnPeople(idxTracksToSave) = legTrackOnPeopleOld;
tracker.peopleTracks = peopleTracks;
tracker.peopleFreeID = freeID;

end

