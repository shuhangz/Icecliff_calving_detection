function [clusterVolumeList,clusterPointCloudList,s] = calculateVolume(s,settings,varargin)
%CALCULATEVOLUME 此处显示有关此函数的摘要
%   此处显示详细说明
narginchk(2,3)
if (nargin == 3 && isa(varargin{1},'pointCloud'))
    pointcloudValidSpace = varargin{1};
end
%% convert lasRead structure to MATLAB pointCloud
xyzPoints = [s.record.x s.record.y s.record.z];
originCloudInd = cast(s.record.original_cloud_index,'logical');

try
    normals = [s.record.normalx s.record.normaly s.record.normalz];
catch
    normals = [s.record.normal_x s.record.normal_y s.record.normal_z];
end

% colors = [s.record.red s.record.green s.record.blue];
% colors = typecast(colors,'uint8');
if isempty(normals)
    ptCloud = pointCloud(xyzPoints);
else
    ptCloud = pointCloud(xyzPoints,'Normal',normals);
end

%% Cluster point cloud based on distance
minDistance = settings.MIN_DIST_CLUSTER;
[labels,numClusters] = pcsegdist(ptCloud,minDistance);
labels = cast(labels,'uint16');
% pcshow(ptCloud.Location,labels)
% colormap(hsv(numClusters))
% title('Point Cloud Clusters')

numRecord = s.variable_length_records.record_length_after_header / 192;
s.variable_length_records.record_length_after_header = 192*(numRecord+1);
s.variable_length_records.value(numRecord+1)=s.variable_length_records.value(1);
s.variable_length_records.value(numRecord+1).name = 'cluster_label';
s.record.cluster_label = labels;
% LASwrite(s,fullfile(folder,strcat(saveFilePrefix,'-cluster.las')),'version',14);

%% clusters validity check
clusterPointCloudList = pointCloud.empty;
numValidClusters = 0;
for i=1:numClusters
    clusterPoint = pointCloud(xyzPoints(labels==i,:), 'Normal', normals(labels==i,:));
    originalInd = originCloudInd(labels==i);
    
    % check percentage of pre and post point,keep clusters that contains
    % both pre- and post- points
    % originalInd:(0--front--before, 1--back--after)
    if settings.FILTER_METHOD_USED(1)
        percentage = sum(originalInd)/clusterPoint.Count;
        if (percentage < settings.PERCENTAGE_THRESHOLD || percentage > (1-settings.PERCENTAGE_THRESHOLD))
            continue;
        end
    end
    
    % remove outliers (cluster whose projection to XY plance is too large)
    if settings.FILTER_METHOD_USED(2)
        
        numBinsX = round((clusterPoint.XLimits(2)-clusterPoint.XLimits(1)));
        numBinsY = round((clusterPoint.YLimits(2)-clusterPoint.YLimits(1)));
        if (numBinsX <= 0 || numBinsY <=0)
            continue;
        end
        indices = pcbin(clusterPoint,[numBinsX numBinsY 1]);
        densityGrid = cellfun(@(c) ~isempty(c),indices);
        if (sum(sum(densityGrid)) > settings.MAX_XYPLANE_AREA)
            continue;
        end
    end
    
    % remove outliers (clusters that fits a plane)
    if settings.FILTER_METHOD_USED(3)
        
        referenceVector = [0,0,1];
        [~, inliers, ~, meanError] = pcfitplane(clusterPoint, 3*settings.MIN_DIST_PLANEFIT,referenceVector);
        inlierPercentage = numel(inliers)/clusterPoint.Count;
        
        if (inlierPercentage > .5 && meanError < .25)
            %         fprintf("exclude cluster #%d, its fit error:%d \n",i,mean(meanError));
            continue;
        end
    end
    
    % remove outliers (floating ice)
    if settings.FILTER_METHOD_USED(4)
        
        clusterPointBefore = select(clusterPoint,originalInd==0);
        clusterPointAfter = select(clusterPoint,originalInd==1);
        [~, inliersAfter, ~, meanErrorAfter] = pcfitplane(clusterPointAfter, settings.MIN_DIST_PLANEFIT,referenceVector);
        [~, inliersBefore, ~, meanErrorBefore] = pcfitplane(clusterPointAfter, settings.MIN_DIST_PLANEFIT,referenceVector);
        inlierPercentageAfter = numel(inliersAfter)/clusterPointAfter.Count;
        inlierPercentageBefore = numel(inliersBefore)/clusterPointBefore.Count;
        
        if (inlierPercentageAfter > .5 && (inlierPercentageBefore < inlierPercentageAfter))
            continue;
        end
    end
    
    % keep clusters inside the valid space
    if (settings.FILTER_METHOD_USED(5))
        boundingBox = [clusterPoint.XLimits, clusterPoint.YLimits, clusterPoint.ZLimits];
        indices = findPointsInROI(pointcloudValidSpace,boundingBox);
        if length(indices) < 10
            continue;
        end
        
    end
    
    
    numValidClusters = numValidClusters + 1;
    % write cluster number to the 'intensity' field
    clusterPoint.Intensity = repmat(numValidClusters,clusterPoint.Count,1);
    clusterPointCloudList=[clusterPointCloudList;clusterPoint];
    
end
assert(numel(clusterPointCloudList)>0);




%% calculate volume of each clusters, using alphashape
clusterVolumeList = zeros(numel(clusterPointCloudList),1);
for i=1:numel(clusterPointCloudList)
    xyzPoints = clusterPointCloudList(i).Location;
    % regionThreshold = 5, as minimum volume of a region
    shp = alphaShape(xyzPoints,2*settings.MIN_DIST_CLUSTER,'RegionThreshold',5); % alpha radius should be tuned
    %     if shp.numRegions > 1
    %         shp.plot;
    %         error('multiple regions!');
    %     end
    
    shp.RegionThreshold = 1; % only the first region
    clusterVolume = volume(shp);
    clusterVolumeList(i) = clusterVolume;
end

end

