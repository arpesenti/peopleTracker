function deleteTracker( tracker )
%DELETETRACKER Function to clean the tracker state when not needed anymore
% 
% Input:
%   tracker = tracker object to be deleted as obtained by setupTracker
%
%See also SETUPTRACKER, TRACKPEOPLE

if strcmp(tracker.sourceType,'oni')
    % close context of .oni file
    mxNiUpdateContext(tracker.sensorHandle);
    mxNiDeleteContext(tracker.sensorHandle);
elseif strcmp(tracker.sourceType,'ros')
    % shutdown ROS node
    tracker.node.shutdown();
elseif strcmp(tracker.sourceType,'oniLive')
    % close context of the sensor
    mxNiUpdateContext(tracker.sensorHandle);
    mxNiDeleteContext(tracker.sensorHandle);
end

end

