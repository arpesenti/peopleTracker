% add the library to the current path
addpath '..'

% flags for enabling plots - turn to false for time efficiency
enablePlotPhoto = false; % plot of colored people
enablePlotMap = false; % plot of 2D map top-view

% oni files
videoFilename = 'exampleVideo.oni';
sourceType = 'oni';

vidLen = 45; % number of frames to process - restart from beginning if video ends beforehand

% setup tracker
tracker = setupTracker(sourceType, videoFilename, enablePlotPhoto, enablePlotMap);

ts = zeros(vidLen,1);
for i=1:vidLen
    fprintf('%d/%d\n',i,vidLen);

    tic
    % update tracking with current frame information
    [people, tracker] = trackPeople(tracker);
    ts(i) = toc;
end

% these times include also plotting time (if enabled)
fprintf('Mean - time per frame: %f\tfps: %f\n', mean(ts), 1/mean(ts));
fprintf('Min - time per frame: %f\tfps: %f\n', min(ts), 1/min(ts));
fprintf('Max - time per frame: %f\tfps: %f\n', max(ts), 1/max(ts));

% delete tracker
deleteTracker(tracker);

