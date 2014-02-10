% add the library to the current path
addpath '..'

% flags for enabling plots
enablePlotPhoto = true; % plot of colored people
enablePlotMap = true; % plot of 2D map top-view

% live from sensor
sourceType = 'oniLive';
numberFrameToProcess = 300; % number of frames to process, then stop

tracker = setupTracker(sourceType, [], enablePlotPhoto, enablePlotMap);

for i=1:numberFrameToProcess
     % update tracking with current frame information
    [people, tracker] = trackPeople(tracker);  
end

% delete tracker
deleteTracker(tracker);