function [ tracker ] = setupTracker( sourceType, uri, enablePlotPhoto, enablePlotMap, enableOdometry )
%SETUPTRACKER Initialize the people tracker, choosing source type and if
%             enable plots.
%
%Inputs:
%   sourceType = source of the RGB-D data, one of the following:
%                - 'oni': OpenNI recorded video file
%                - 'oniLive': OpenNI compatible sensor directly connected to
%                         the PC.
%                - 'ros': the people tracker acts as ROS node. It subscribes
%                         to the RGB-D sensor topics (/camera/depth_registered/image_raw
%                         and /camera/rgb/image_raw) and to the
%                         odometry topic (/odom), and publish the found
%                         people (/people, with message type people_msg/People)
%   uri = uri where to search for data, one of the following corresponding
%         to the chosen source type:
%         - 'oni': string containing the path to the .oni file
%         - 'oniLive': string containing the path to a valid OpenNI XML
%                      sensor configuration (see openni/SensorConfig.xml ad
%                      example)
%         - 'ros': string containig address and port of the ROS core
%   enablePlotPhoto = (optional) boolean to enable the point cloud plot
%                     with overlay of tracked people, for debugging
%                     purposes. Default is false.
%   enablePlotMap = (optional) boolean to enable the plot of a top view map
%                   containing the tracked people, for debugging purposes. 
%                   Default is false.
%   enableOdometry = (optional) boolean to enable the reading of odometry
%                    from ros topic /odom. Default is false.
%
%Output:
%   tracker = object containing the state of the tracker
%
%See also TRACKPEOPLE, DELETETRACKER

% --- PARAMETERS ---
% Parameters single candidates tracking 
tracker.legSigmaZ = 0.02; % position measure uncertainty (m)
tracker.legSigmaP = 0.2; % leg probability measure uncertainty
tracker.legSigmaAcc = 6; % model uncertainty, taking into account accelerations (m/s^2)

% Parameters people tracking 
tracker.peopleSigmaZ = 0.05; % position measure uncertainty (m)
tracker.peopleSigmaP = 0.2; % leg probability measure uncertainty
tracker.peopleSigmaAcc = 6; % model uncertainty, taking into account accelerations (m/s^2)
tracker.peopleDistThreshold = 75; % maximum distance between leg expansions, for leg associations
tracker.legProbabilityThreshold = 0.8; % probability threshold over which a candidates is considere a leg

% Parameter to specify how often searching for a floor plane
tracker.refreshIntervalFloorPlane = 0.05; %seconds

% Parameter to simulate different frame rates in recorded oni videos
tracker.fps = 30;

% Path to the trained svm classifier (libsvm)
svmPath = 'svm.mat';

% Enable plot of floor plane
tracker.enablePlotFloor = false;

% floor plane tolerance in mm
tracker.floorPlaneTolerance = 2;

% sensor upside down (vertical flip)
tracker.upsideDown = false;

% mirrored data (horizontal flip)
tracker.mirrored = false;

% ROS topics
rgbTopic = '/camera/rgb/image_raw'; % message type: sensor_msgs/Image encoding: yuv422
depthTopic = '/camera/depth_registered/image_raw'; % message type: sensor_msgs/Image encoding: 16UC1
odometryTopic = '/odom'; % message type: nav_msgs/Odometry

% record video of processed point cloud (at least one plot must be enabled)
tracker.recordVideo = false;

% -----------------
libraryPath = fileparts(which('setupTracker'));
addpath([libraryPath filesep 'libsvm']);
addpath([libraryPath filesep 'openni']);
addpath([libraryPath filesep 'ros_matlab_bridge']);

svmStruct = load(svmPath);
tracker.svm = libsvmreadmodel(svmStruct.svm);

tracker.sourceType = sourceType;

if nargin < 4
    tracker.enablePlotPhoto = false;
    tracker.enablePlotMap = false;
else
    tracker.enablePlotPhoto = enablePlotPhoto;
    tracker.enablePlotMap = enablePlotMap;
end

if nargin < 5
    tracker.enableOdom = false;
else
    tracker.enableOdom = enableOdometry;
end

if tracker.enablePlotPhoto && tracker.recordVideo
   tracker.writerObjPhoto = VideoWriter('videoPhoto.mp4','MPEG-4');
   tracker.writerObjPhoto.Quality = 100;
   open(tracker.writerObjPhoto);
end

if tracker.enablePlotMap && tracker.recordVideo
   tracker.writerObjMap = VideoWriter('videoMap.mp4','MPEG-4');
   tracker.writerObjMap.Quality = 100;
   open(tracker.writerObjMap);
end

if tracker.enablePlotPhoto || tracker.enablePlotMap
    tracker.figure = figure;
    nColors  = 50;
    colors=jet(nColors);
    tracker.colors = colors(randperm(nColors, nColors),:);
    if tracker.enablePlotPhoto && tracker.enablePlotMap
        tracker.hPhoto = subplottight(1,2,1);
        tracker.hMap = subplottight(1,2,2);
        set(tracker.hMap,'Color',[0.4 0.4 0.4]);
        windowSize = [0 100 1280 480];
        radius = 200;
        tracker.triangle = createTriangle(0.8,1.2*radius);
        tracker.smallCircle = createCircle(100);
        tracker.circle = createCircle(radius);
        axis(tracker.hMap,[-2 2 .5 3.5]);
        set(tracker.hMap, 'xtick',[]);
        set(tracker.hMap, 'ytick',[]);
    elseif tracker.enablePlotPhoto
        tracker.hPhoto = subplottight(1,1,1);
        windowSize = [0 100 640 480];
        radius = 200;
        tracker.triangle = createTriangle(0.8,1.2*radius);
        tracker.smallCircle = createCircle(100);
        tracker.circle = createCircle(radius);
    elseif tracker.enablePlotMap
        tracker.hMap = subplottight(1,1,1);
        windowSize = [0 100 640 480];
        set(tracker.hMap,'Color',[0.4 0.4 0.4]);
        axis(tracker.hMap,[-2.5 2.5 .5 3.5]);
        set(tracker.hMap, 'xtick',[]);
        set(tracker.hMap, 'ytick',[]);
        radius = 200;
        tracker.triangle = createTriangle(0.8,1.2*radius);
        tracker.smallCircle = createCircle(100);
        tracker.circle = createCircle(radius);
        set(tracker.figure,'InvertHardCopy','off')
    end
    set(tracker.figure, 'Toolbar', 'none');
    set(tracker.figure, 'Position', windowSize);
end

function [depth, rgb, tracker] = updateFromOni(sensorHandle, tracker)
    % read data from .oni file
    depth = permute(mxNiDepth(sensorHandle), [2 1]);
    rgb = [];
    if tracker.enablePlotPhoto
        rgb = permute(mxNiPhoto(sensorHandle), [3 2 1]);
    end
    
    if tracker.upsideDown
        depth = depth(end:-1:1,:);
        rgb = rgb(end:-1:1,:,:);
    end
    
    if tracker.mirrored
        depth = depth(:,end:-1:1);
        rgb = rgb(:,end:-1:1,:);
    end
    
    % if parameter tracker.fps different from the maximum (30) simulate
    % lower frame rate
    maxFps = 30;
    tracker.frameNumDecimal = tracker.frameNumDecimal + maxFps/tracker.fps;
    skipNumber = round(tracker.frameNumDecimal) - tracker.frameNum;

    for i=1:skipNumber
        mxNiUpdateContext(sensorHandle);
    end
    
    % update timestamp of the current frame
    tracker.oldTimestamp = tracker.currentTimestamp;
    tracker.currentTimestamp = tracker.currentTimestamp + skipNumber*1/maxFps; % seconds
    tracker.frameNum = tracker.frameNum + skipNumber;
end

function [depth, rgb, tracker] = updateFromOniLive(sensorHandle, tracker)
    % read data from an OpenNI compatible sensor
    mxNiUpdateContext(sensorHandle);
    
    depth = permute(mxNiDepth(sensorHandle), [2 1]);
    rgb = [];
    if tracker.enablePlotPhoto
        rgb = permute(mxNiPhoto(sensorHandle), [3 2 1]);
    end
    
    if tracker.upsideDown
        depth = depth(end:-1:1,:);
        rgb = rgb(end:-1:1,:,:);
    end
    
    if tracker.mirrored
        depth = depth(:,end:-1:1);
        rgb = rgb(:,end:-1:1,:);
    end
    
    % update timestamp of the current frame
    tracker.oldTimestamp = tracker.currentTimestamp;
    tracker.currentTimestamp = toc(uint64(1)); % seconds
end

function [depth, rgb, tracker] = updateFromRos(tracker)
    % read data from ROS
    timeout = 1;
    depthMessage = [];
    rgbMessage = [];
    odometryMessage = [];
    if tracker.enablePlotPhoto
        while isempty(depthMessage) || isempty(rgbMessage) 
            % wait for rgb-d and odomotery messages
            depthMessage = tracker.depthSubscriber.takeMessage(timeout);
            rgbMessage = tracker.rgbSubscriber.takeMessage(timeout);
            if tracker.enableOdom
                odometryMessage = tracker.odometrySubscriber.takeMessage(timeout);
            end
        end
     
%         % rgb jpeg decompression
%         jImg = javax.imageio.ImageIO.read(java.io.ByteArrayInputStream(typecast(rgbMessage.data,'uint8')));
%         h = jImg.getHeight;
%         w = jImg.getWidth;
%         p = reshape(typecast(jImg.getData.getDataStorage, 'uint8'), [1,w,h]);
%                 
%         % demosaic of Bayer GRBG encoded image
%         rgb = demosaic(transpose(reshape(p(1,:,:), [w,h])), 'grbg');

         w = rgbMessage.width;
         h = rgbMessage.height;
         
         % from yuv422 to rgb
         temp = zeros(w*h,3,'uint8');
         temp(:,1) = typecast(rgbMessage.data(2:2:end),'uint8');
         temp(1:2:end,2) = typecast(rgbMessage.data(1:4:end),'uint8');
         temp(2:2:end,2) = typecast(rgbMessage.data(1:4:end),'uint8');
         temp(1:2:end,3) = typecast(rgbMessage.data(3:4:end),'uint8');
         temp(2:2:end,3) = typecast(rgbMessage.data(3:4:end),'uint8');
         
         yuvTorgb = [1 0 1.13983; 1 -0.39465 -0.58060; 1 2.03211 0]'; 
         temp = cast(temp,'double');
         temp(:,[2 3]) = temp(:,[2 3]) - 128;
         temp = temp * yuvTorgb ;
         temp = cast(temp, 'uint8');
         rgb = cat(3,reshape(temp(:,1), [w h])', reshape(temp(:,2),[w h])', reshape(temp(:,3),[w h])');
         
         
         % decode bgr8 encoding 8UC3 image
         %rgb = cat(3,reshape(typecast(rgbMessage.data(3:3:end),'uint8'),[w h])', reshape(typecast(rgbMessage.data(2:3:end),'uint8'),[w h])', reshape(typecast(rgbMessage.data(1:3:end),'uint8'),[w,h])');
 
       
    else
        rgb = [];
        while isempty(depthMessage) 
            % wait for depth and odomotery messages
            depthMessage = tracker.depthSubscriber.takeMessage(timeout);
            if tracker.enableOdom
                odometryMessage = tracker.odometrySubscriber.takeMessage(timeout);
            end
        end
    end

   
    % decode 16UC1 depth   
    w = depthMessage.width;
    h = depthMessage.height;
    depth = typecast(depthMessage.data, 'uint8');
    depth = reshape(bitor(cast(depth(1:2:end), 'uint16'), bitshift(cast(depth(2:2:end), 'uint16'),8)), w,h)';

    if tracker.upsideDown
        depth = depth(end:-1:1,:);
        rgb = rgb(end:-1:1,:,:);
    end
    
    if tracker.mirrored
        depth = depth(:,end:-1:1);
        rgb = rgb(:,end:-1:1,:);
    end
    
    % read odometry
    if tracker.enableOdom && ~isempty(odometryMessage)
        tracker.pose(1) = odometryMessage.pose.pose.position.x; % x real world coordinate
        tracker.pose(2) = odometryMessage.pose.pose.position.y; % y real world coordinate
        % robot orientation (yaw)
        [tracker.pose(3), ~, ~] = quat2angle([odometryMessage.pose.pose.orientation.w odometryMessage.pose.pose.orientation.x odometryMessage.pose.pose.orientation.y odometryMessage.pose.pose.orientation.z]);
        tracker.pose(3) = tracker.pose(3)-pi/2; % adjust angle between robot odometry and our rotated reference system
    end
    
    % update timestamp
    tracker.oldTimestamp = tracker.currentTimestamp;
    tracker.currentTimestamp = depthMessage.header.stamp.totalNsecs*1e-9; % seconds
    
    tracker.frameNum = tracker.frameNum +1;
end

function publishROS(tracker, people)
    % publish tracked people position and velocity over ROS
    % topic: /people
    % message type: people_msg/People
    
    msg = org.ros.message.people_msg.People();
    
    if isempty(people)
        msg.numberOfPeople = 0;
%         msg.peopleID = 0; % unique IDs of tracked people 
%         msg.x = 0; % x real world coordinates of people
%         msg.y = 0; % y real world coordinates of people
%         msg.vx = 0; % x compoment of absolute velocity of people
%         msg.vy = 0; % y compoment of absolute velocity of people
    else
        numPeople = length(people);
        msg.numberOfPeople = numPeople; % number of people currently tracked
        positions = cat(1,people.peoplePosition);
        velocities = cat(1,people.peopleVelocity);
        msg.peopleID = cat(1,people.id); % unique IDs of tracked people 
        msg.x = positions(:,1); % x real world coordinates of people
        msg.y = positions(:,2); % y real world coordinates of people
        msg.vx = velocities(:,1); % x compoment of absolute velocity of people
        msg.vy = velocities(:,2); % y compoment of absolute velocity of people
    end
    
    tracker.peoplePublisher.publish(msg);
end

if strcmp(sourceType,'oni')
    % initialize OpenNI context for record video
    CONFIG_XML_PATH='openni/SensorConfig.xml';
    tracker.sensorHandle = mxNiCreateContext(CONFIG_XML_PATH, uri);
    tracker.update = @(tracker) updateFromOni(tracker.sensorHandle, tracker);
elseif strcmp(sourceType,'ros')
    % initialize ROS node named PeopleTracker
    run([libraryPath '/ros_matlab_bridge/jmb_init']);
    tracker.node = jmb_init_node('PeopleTracker', uri);
    tracker.depthSubscriber = edu.ucsd.SubscriberAdapter(tracker.node, depthTopic,'sensor_msgs/Image');
    tracker.rgbSubscriber = edu.ucsd.SubscriberAdapter(tracker.node, rgbTopic,'sensor_msgs/Image');
    
    if tracker.enableOdom 
        tracker.odometrySubscriber = edu.ucsd.SubscriberAdapter(tracker.node, odometryTopic,'nav_msgs/Odometry');
    end
    tracker.peoplePublisher = tracker.node.newPublisher('/people','people_msg/People');
    tracker.publish = @(people) publishROS(tracker, people);
    tracker.update = @(tracker) updateFromRos(tracker);
elseif strcmp(sourceType,'oniLive')
    % initialize OpenNI context for live video
    CONFIG_XML_PATH='openni/SensorConfig.xml';
    tracker.sensorHandle = mxNiCreateContext(CONFIG_XML_PATH);
    tracker.update = @(tracker) updateFromOniLive(tracker.sensorHandle, tracker);
else
    error('Invalid source type. Supported types are oni, ros and oniLive. See setupTracker documentation.');
end

% initialize state of the tracker
tracker.legTracks = [];
tracker.peopleTracks = [];
tracker.legFreeID = 1;
tracker.peopleFreeID = 1;
tracker.oldTimestamp = 0;
tracker.currentTimestamp = 0;
tracker.frameNum = 0;
tracker.frameNumDecimal = 0; % to be used when a fps different from 30 is used
tracker.floorPlane = [0 1 0 250]; % initialize as if camera is perfectly horizontal at 25cm above the floor
tracker.pose = zeros(3,1);
tracker.floorPlaneTimestamp = -Inf;

end

function triangle = createTriangle(scale,offset)
% build a set of 3D point representing a triangle, for plotting reasons.
b1 = [0; 0].*scale;
b2 = [0; 250].*scale;
h = [300; 125].*scale;
triangle = [];
resH = 5;
resV = 10;
step = (h(2)-b1(2))/(h(1)-b1(1))*resH;
jstart = b1(2);
jend = b2(2);
thickness = linspace(5,10,1);
for i=b1(1):resH:h(1)
    for j=jstart:resV*(1-i/h(1))+0.1:jend
        triangle = [triangle [repmat([i; j], 1, length(thickness)); thickness]];
    end
    jstart = jstart+step;
    jend = jend-step;
end

centroid = (b1+b2+h)./3;
triangle(1:2,:) = triangle(1:2,:) - repmat(centroid-[offset;0],1,size(triangle,2));
end

function circle = createCircle(radius)
n = 30;
circle = zeros(3,ceil(0.1*2*radius*pi)*n);
count = 0;
for ro=linspace(0,radius,n)
    startingAngle = rand(1)*2*pi;
    for theta=linspace(0+startingAngle,2*pi+startingAngle,ceil(0.1*2*ro*pi))
        count = count + 1;
        circle(:,count) = [ro*cos(theta+ro); ro*sin(theta+ro); 7.5];
    end
end
circle(:,count+1:end) = [];
end




