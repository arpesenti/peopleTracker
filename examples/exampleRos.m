% add the library to the current path
addpath '..'

% flags for enabling plots
enablePlotPhoto = true; % plot of colored people
enablePlotMap = true; % plot of 2D map top-view
enableOdometry = false; % read odometry on /odom 

% ros parameters
uri = 'http://localhost:11311'; % ROS_MASTER_URI
sourceType = 'ros';

tracker = setupTracker(sourceType, uri, enablePlotPhoto, enablePlotMap, enableOdometry);

numberFramesToProcess = 100;
for i=1:numberFramesToProcess   
    % update tracking with current frame information
    [people, tracker] = trackPeople(tracker);    
end

deleteTracker(tracker);