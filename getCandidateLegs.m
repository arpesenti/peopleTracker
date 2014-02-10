function [ candidates ] = getCandidateLegs( points, colors )
%GETCANDIDATELEGS search for clusters of points which can be candidate to
%be legs and return them in a vector of candidate structs.
%   
%Input:      
%   points = 3xN matrix of N points of the 3D point cloud
%   colors = 3xN matrix of N rgb colors corresponding to the points
%
%Output:
%   candidates = vector of M candidate struct (see below in the code the 
%                definition of the candidate struct)
    
    if isempty(colors)
        enabledColor = false;
    else
        enabledColor = true;
    end

    % cut the point cloud at foot and whole person levels
    footCutHeight = 210; % mm
    personCutHeight = 2000; % mm
    footCutBase = 30; % mm
    legCutBase = 10; % mm
    
    idxVoxelGridPoints = find(points(3,:) > legCutBase & points(3,:) < personCutHeight);
    voxelGridPoints = points(:,idxVoxelGridPoints);
    
    idxFootCutPoints = find(voxelGridPoints(3,:) > footCutBase & voxelGridPoints(3,:) < footCutHeight);
    footCutPoints = voxelGridPoints(:,idxFootCutPoints);
    
    voxelGridPoints(:,idxFootCutPoints) = [];    
    
    if enabledColor
        % cut the corresponding colors
        voxelGridColors = colors(:,idxVoxelGridPoints);
        footCutColors = voxelGridColors(:,idxFootCutPoints);
        voxelGridColors(:,idxFootCutPoints) = [];
    else
        voxelGridColors = [];
    end
        
    if isempty(footCutPoints)
        % no foot points in point cloud
        candidates = [];
        return; 
    end
    
      
    % apply DBSCAN on foot level
    nNeigh = 5; % number of neighbors
    radius = 40; % neighborhood radius in mm
    
    [class, type] = dbscanClustering(footCutPoints, nNeigh, radius);
    
    nClusters = max(class);
    
    if nClusters < 1
        candidates = [];
        return;
    end
        
    % candidate struct:
    %     rectangleOnFloor = vertices of the rectangle containing the foot 
    %                        candidate projection onto the floor
    %     footPoints = 3xF matrix of 3D foot points (foot level) of the candidate
    %     legPoints = 3xL matrix of 3D leg points (from floor to knee
    %                 level) of the candidate
    %     allPoints = 3xA matrix of 3D whole points (from floor upwards until 
    %                 possible) of the candidate
    %     legPointsNormalized = leg points rotated and translated
    %     legColors = 3xL matrix of rgb colors corresponding to legPoints
    %                 (it is empty if plotPhoto is not enabled)
    %     footColors = 3xF matrix of rgb colors corresponding to footPoints
    %                  (it is empty if plotPhoto is not enabled)
    %     centroid = 2D centroid of footPoints projected on floor
    %     silhouette = silhouette of the candidate. It's the matrix
    %                   containing the rectified depth image of the candidate leg
   

    % initialize candidates struct array
    candidates(1:nClusters) = struct('rectangleOnFloor',zeros(4,2),'footPoints',[],'legPoints',[],'allPoints',[],'legPointsNormalized',[], 'legColors', [], 'footColors', [], 'centroid', [], 'silhouette',[]);
    
    % rough filtering of clusters based on their dimensions
    clusterOK(1:nClusters) = 0;
    candidateOK(1:nClusters) = 0;
    centroidsOnFloor = zeros(2,nClusters);
      
    minTol = 50; % min foot size in mm
    maxTol = 600; % max foot size in mm
    minNumPoints = 20; % minimum number of points to consider a cluster
        
    for i=1:nClusters
        footPoints = footCutPoints(:,class == i);
                
        if size(footPoints, 2)<minNumPoints || rank(footPoints) < 3
            % cluster with too few points
            clusterOK(i) = 0;
            continue;
        end
        
        % find bounding rectangle on floor
        rectangleOnFloor =  minBoundingBox(footPoints(1:2,:));
        
        l1 = sqrt(sum((rectangleOnFloor(:,1)-rectangleOnFloor(:,2)).^2));
        l2 = sqrt(sum((rectangleOnFloor(:,2)-rectangleOnFloor(:,3)).^2));
        
        if l1<minTol && l2<minTol
            % cluster too small
            clusterOK(i) = 0;
            continue;
        end
        
        clusterOK(i) = 1;
        
        if l1<maxTol && l2<maxTol && l1>minTol && l2>minTol
            % candidate has right dimensions
            candidateOK(i) = 1;            
        end
        
        centroidsOnFloor(:,i) = mean(footPoints(1:2,:),2);
        candidates(i).centroid = centroidsOnFloor(:,i);
        candidates(i).rectangleOnFloor = rectangleOnFloor;
        candidates(i).footPoints = footPoints;
        if enabledColor
            candidates(i).footColors = footCutColors(:,class == i);
        end
    end
    candidates(clusterOK == 0) = [];
    centroidsOnFloor(:,clusterOK == 0) = [];
    candidateOK(clusterOK == 0) = [];
    
    candidates(candidateOK == 0) = [];
    
    % all the candidates centroids will be used for determining Voronoi regions for the expansion 
    centroidsOnFloor = [centroidsOnFloor(:, candidateOK == 1) centroidsOnFloor(:, candidateOK == 0)];
    
    if isempty(candidates)
        candidates = [];
        return;
    end
    
    
    % apply expansion of candidates upwards
    candidates = expandCandidates(candidates, centroidsOnFloor, voxelGridPoints, voxelGridColors);

end