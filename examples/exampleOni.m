% add the library to the current path
addpath '..'

% flags for enabling plots
enablePlotPhoto = true; % plot of colored people
enablePlotMap = true; % plot of 2D map top-view

% oni files
videoFilename = 'exampleVideo.oni';
sourceType = 'oni';

vidLen = 45; % number of frames to process - restart from beginning if video ends beforehand

% setup tracker
tracker = setupTracker(sourceType, videoFilename, enablePlotPhoto, enablePlotMap);

for i=1:vidLen
    fprintf('%d/%d\n',i,vidLen);

    % update tracking with current frame information
    [people, tracker] = trackPeople(tracker);    
end

% delete tracker
deleteTracker(tracker);