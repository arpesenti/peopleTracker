function [ people, tracker ] = trackPeople( tracker )
%PEOPLETRACKER Main function for people tracking. New RGB-D data is fetched from
%the selected source, processed and an array of tracked people is returned.
%
%Input:
%   tracker = tracker object as obtained by setupTracker (first frame) or returned by
%             this function (subsequents frames)
%
%Outputs:
%   people = array of person structs (see below)
%   tracker = updated tracker object to be passed to a new call of this function
%
%Person struct: 
%   id = unique ID corresponding to the tracked person. If 0 previously unseen person
%   peoplePosition = 1x2 vector containing person barycenter position in
%                    absolute or relative coordinates in meters, depending if odometry is 
%                    available or not.
%   peopleVelocity = 1x2 vector containing person barycenter absolute or relative
%                    velocity in meters per second, depending if odometry is available or not.
%   peopleHeading = angle representing person movement direction
%   legPositions = nx2 matrix containing positions of person legs in absolute 
%                  or relative coordinates in meters, depending if odometry
%                  is available or not. The person could have associated 0,
%                  1 or 2 legs.
%   legVelocities = nx2 matrix containing absolute or relative velocities
%                   in meters per second, depending if odometry is available or not.
%                   The person could have associated 0, 1 or 2 legs.
%   legVisible = nx1 vector containg 1 if the corresponding leg is
%                visible in the current frame, 0 otherwise.
%
%See also SETUPTRACKER, DELETETRACKER
  

legProbabilityThreshold = tracker.legProbabilityThreshold;

% --- read RGB-D data from the source ---
[depth, rgb, tracker] = tracker.update(tracker);

% --- point cloud downsampling using Voxel Grid Filter ---
[points, colors] = getVoxelGrid(depth, rgb);

% --- search for floor plane, if refreshIntervalFloorPlane has elapsed since
% the last floor update ---
if tracker.currentTimestamp - tracker.floorPlaneTimestamp > tracker.refreshIntervalFloorPlane
    updatedPlane =  getGroundPlane(points,20,tracker.floorPlaneTolerance);
    if ~isempty(updatedPlane)
       tracker.floorPlane = updatedPlane;
       tracker.floorPlaneTimestamp = tracker.currentTimestamp;
    end    
end

% --- transform the point cloud to a more pratical reference system ---
[rotatedPoints,~,Rinv] = rotatePointCloud(points, tracker.floorPlane);

% --- search candidates legs in the point cloud ---
candidates = getCandidateLegs(rotatedPoints, colors);
tracker.candidates = candidates;

% --- extract features and predict candidates leg probabilities ---
probEstimates = predictClass(tracker.svm, candidates);
tracker.probEstimates = probEstimates;

% --- tracking of single candidates ---
[candidatesTrack, tracker] = candidatesTrackerPDAF(candidates, probEstimates, tracker);
tracker.candidatesTrack = candidatesTrack;

% --- tracking of people ---
[ legTrackOnPeople, tracker] = peopleTrackerPDAFOnLegTracks( candidates, candidatesTrack, tracker );

% --- build people struct ---
% consider robot yaw and position, if odometry is available
yaw = tracker.pose(3);
R = [cos(yaw) sin(yaw); -sin(yaw) cos(yaw)];
robotPosition = tracker.pose(1:2);

numLegTracks = length(tracker.legTracks);
numPeopleTracks = length(tracker.peopleTracks); 

% initialize person struct (see the function help for the description)
people(1:numLegTracks+numPeopleTracks) = struct('id', 0, 'peoplePosition', zeros(1,2), 'peopleVelocity', zeros(1,2), 'peopleProbability', 0, 'peopleHeading', 0, 'legPositions', zeros(2,2), 'legVelocities', zeros(2,2), 'legVisible', zeros(2,1));

% fill person struct for every peopleTracks
oldPeopleToKill = [];
for i=1:numPeopleTracks
    
    % person position, velocity and heading, eventually considering odometry
    people(i).id = tracker.peopleTracks(i).id;
    people(i).peoplePosition = (R*(tracker.peopleTracks(i).prediction.x(1:2)-robotPosition))';
    people(i).peopleVelocity = (R*tracker.peopleTracks(i).avgSpeed)';
    people(i).peopleProbability = (tracker.peopleTracks(i).prediction.x(5)+1)/2;
    people(i).peopleHeading = tracker.peopleTracks(i).heading-yaw;
    
    % find legTracks associated with the current person
    legsIdx = find(legTrackOnPeople == i);
    if isempty(legsIdx)
      % no legs associated
      leg1idx = 0;
      leg2idx = 0;
    elseif length(legsIdx) == 1
      % only one leg associated
      leg1idx = legsIdx(1);
      leg2idx = 0;
    elseif length(legsIdx) == 2
      % two legs associated
      leg1idx = legsIdx(1);
      leg2idx = legsIdx(2);
    else
      % more than two legs associated, choose the two with higher probability
      predictions = cat(2,tracker.legTracks(legsIdx).prediction);
      states = cat(2,predictions.x);
      probabilities = (states(5,:)+1)/2;
      [~, idxSort] = sort(probabilities,2,'descend');
      legsIdx = legsIdx(idxSort);
      leg1idx = legsIdx(1);
      leg2idx = legsIdx(2);
    end
    
    if leg1idx ~= 0
        % fill person struct with position and velocity of the first leg
        if ~isempty(find(candidatesTrack == leg1idx))
            people(i).legVisible(1) = 1;
        end
        leg1 = tracker.legTracks(leg1idx);
        people(i).legPositions(1,:) = (R*(leg1.prediction.x(1:2)-robotPosition))';
        people(i).legVelocities(1,:) = leg1.prediction.x(3:4)';
    else
        % no first leg associated
        people(i).legPositions(1,:) = [];
        people(i).legVelocities(1,:) = [];
        people(i).legVisible(1) = [];
    end
    if leg2idx ~= 0
        % fill person struct with position and velocity of the second leg
        if ~isempty(find(candidatesTrack == leg2idx))
            people(i).legVisible(end) = 1;
        end
        leg2 = tracker.legTracks(leg2idx);
        people(i).legPositions(end,:) = (R*(leg2.prediction.x(1:2)-robotPosition))';
        people(i).legVelocities(end,:) = leg2.prediction.x(3:4)';
    else
        % no second leg associated
        people(i).legPositions(end,:) = [];
        people(i).legVelocities(end,:) = [];
        people(i).legVisible(end) = [];
        if isempty(people(i).legVisible)
            people(i).legVisible = [];
        end
    end
end

people(oldPeopleToKill) = [];

% consider also not associated legs as unseen people
notUsedLegTracks = find(legTrackOnPeople < 1); 
numPeople = numPeopleTracks - length(oldPeopleToKill);
for i=1:length(notUsedLegTracks)
    leg = tracker.legTracks(notUsedLegTracks(i));
    if (leg.prediction.x(5)+1)/2 > legProbabilityThreshold
        % if leg has an high probability, create a new person struct with id 0
        numPeople = numPeople + 1;
        people(numPeople).peoplePosition = (R*(leg.prediction.x(1:2)-robotPosition))';
        people(numPeople).peopleVelocity = (R*leg.prediction.x(3:4))';
        people(numPeople).peopleProbability = (leg.prediction.x(5)+1)/2;
        people(numPeople).legPositions = people(numPeople).peoplePosition;
        people(numPeople).legVelocities = people(numPeople).peopleVelocity;
        if ~isempty(find(candidatesTrack == notUsedLegTracks(i)))
            people(numPeople).legVisible = 1;
        end
        people(numPeople).peopleHeading = atan2(people(numPeople).peopleVelocity(2), people(numPeople).peopleVelocity(1)) - yaw;
    end
end
people(numPeople+1:end) = [];

% publish to ROS
if strcmp(tracker.sourceType,'ros')
   tracker.publish(people);
end

% --- plots, if enabled ---
if tracker.enablePlotPhoto || tracker.enablePlotMap
    stillPersonThreshold = 0.1^2;
    legTracks = tracker.legTracks;
    peopleTracks = tracker.peopleTracks;
    
    nColors = size(tracker.colors,1);
    
    figure(tracker.figure);
    if tracker.enablePlotPhoto
        maxSpeed = 2;
        overlayColors = [];
        overlayPoints = [];
        alphaBackground = 1;
        alphaOverlay = 0.5;
        colours = tracker.colors;  
        for j=1:length(candidates)
           if ~isempty(candidatesTrack) && candidatesTrack(j) ~= 0 && legTrackOnPeople(candidatesTrack(j)) ~= 0
              % find to which person the candidate is associated
              candidate = candidates(j);
              peopleIdx = legTrackOnPeople(candidatesTrack(j));
              if peopleIdx<1
                  temp = regexp(num2str(peopleIdx),'\.','split');
                  peopleIdx = str2double(temp{2});
                  person = peopleTracks(peopleIdx);
              else
                  person = peopleTracks(peopleIdx);
              end
              
              % add an overlay to the points of the canidate with the color
              % corresponding to the person
              peopleId = person.id;
              color = colours(mod(peopleId,nColors)+1,:);
              overlayColors = [overlayColors repmat(color',1,size(candidate.allPoints,2))];
              overlayPoints = [overlayPoints cat(2,candidate.allPoints)];
            end
        end
    end
    if tracker.enablePlotMap
        cla(tracker.hMap);
        hold(tracker.hMap,'on');
        
        %plot the field of view on the map
        yMax = 3.5;
        fieldOfView = 57/180*pi;
        xMin = -tan(fieldOfView/2)*yMax;
        xMax = tan(fieldOfView/2)*yMax;
        yMin = 0;
        plot(tracker.hMap, [xMin; 0; xMax], [yMax; yMin; yMax],'w','MarkerSize',5);
    end
    for i=1:length(people)
        if people(i).id ~= 0
            % person color if is a tracked person
            color = cast(floor(255*tracker.colors(mod(people(i).id,nColors)+1,:)),'uint8');
        else
            % green if is a previously unseen person (a leg not associated)
            color = cast([0 255 0],'uint8');
        end
        peopleX = people(i).peoplePosition(1);
        peopleY = people(i).peoplePosition(2);
        vx = people(i).peopleVelocity(1);
        vy = people(i).peopleVelocity(2);
        if tracker.enablePlotMap
            scale = 0.2*1e-3;
            triangleMap = [ [0; 0].*scale [0; 250].*scale [400; 125].*scale [0; 0].*scale];
            triangleMap = triangleMap - repmat(mean(triangleMap(:,1:3),2),1,4);
            colorDouble = cast(color,'double')./255;
            if people(i).id ~= 0
                for k=1:size(people(i).legPositions,1)
                    legX = people(i).legPositions(k,1);
                    legY = people(i).legPositions(k,2);
                    plot(tracker.hMap, legX, legY, 'o', 'MarkerSize', 10, 'MarkerEdgeColor', colorDouble, 'MarkerFaceColor', colorDouble);
                end
                
                % consider heading of the person
                alpha = people(i).peopleHeading;
                if vx^2+vy^2 > stillPersonThreshold
                    rotatedTriangleMap = [cos(alpha) -sin(alpha); sin(alpha) cos(alpha)]*triangleMap;
                    plot(tracker.hMap, peopleX+rotatedTriangleMap(1,:)', peopleY+rotatedTriangleMap(2,:)', 'LineWidth', 6, 'Color', colorDouble);
                    rotatedTriangleMap = 1.9*rotatedTriangleMap;
                    plot(tracker.hMap, peopleX+rotatedTriangleMap(1,:)', peopleY+rotatedTriangleMap(2,:)', 'LineWidth', 2, 'Color', 'white');
                else
                    plot(tracker.hMap, peopleX, peopleY, 'o', 'MarkerSize', 15, 'LineWidth', 3, 'MarkerEdgeColor', 'white', 'MarkerFaceColor', colorDouble);
                end
            else
                plot(tracker.hMap, peopleX, peopleY, 'o', 'MarkerSize', 10, 'MarkerEdgeColor', colorDouble, 'MarkerFaceColor', colorDouble);
            end
        end
        if tracker.enablePlotPhoto
            % consider heading of the person
            alpha = people(i).peopleHeading;
            if people(i).id ~= 0
                circle = tracker.circle;
            else
                circle = tracker.smallCircle;
            end
            circle(1:2,:) = circle(1:2,:) + repmat([peopleX;peopleY]*1000,1,size(circle,2));
            rotatedPoints = [rotatedPoints circle];
            circleColors = repmat(color', 1,size(circle,2))*2;
            colors = [colors circleColors];
            
            if vx^2+vy^2 > stillPersonThreshold && people(i).id ~= 0
               triangle = tracker.triangle;
               triangle(1:2,:) = [cos(alpha) -sin(alpha); sin(alpha) cos(alpha)]*triangle(1:2,:) + repmat([peopleX;peopleY]*1000,1,size(triangle,2));
               rotatedPoints = [rotatedPoints triangle];
               numPointsArrowHead = round(0.8*size(triangle,2));
               triangleColors = [ repmat(color', 1,size(triangle,2)-numPointsArrowHead)*2 repmat([255;255;255], 1,numPointsArrowHead)*2];
               colors = [colors triangleColors];
            end
        end
    end
    if tracker.enablePlotPhoto
        % plot floor plane
        if tracker.enablePlotFloor
           plane = tracker.floorPlane;
           idx = find(abs(plane'*[points; ones(1,size(points,2))]) < sqrt(sum(plane(1:3).^2))*tracker.floorPlaneTolerance);
           colors(1,idx) = 0;
           colors(2,idx) = 255;
           colors(3,idx) = 255;
        end
        
        % generate the image of the point cloud with the specified overlays
        im = plotPC(rotatedPoints, colors, Rinv,overlayPoints,overlayColors,alphaBackground, alphaOverlay);
        image(im,'Parent',tracker.hPhoto);
        if tracker.recordVideo
           im = imdilate(im,strel('disk',1));
           writeVideo(tracker.writerObjPhoto, cast(im*255,'uint8'));
        end
    end
    if tracker.enablePlotMap
        if tracker.recordVideo
            writeVideo(tracker.writerObjMap, getframe(tracker.hMap));
        end
        hold(tracker.hMap,'off');
    end
end

end

