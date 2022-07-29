function [clusterVolumeList,clusterPointCloudList,s] = calculateVolume(s,settings)
%CALCULATEVOLUME 此处显示有关此函数的摘要
%   此处显示详细说明

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

%% keep clusters that contains both pre- and post- points
clusterPointCloudList = pointCloud.empty;
numValidClusters = 0;
for i=1:numClusters
    clusterPoint = pointCloud(xyzPoints(labels==i,:), 'Normal', normals(labels==i,:));
    originalInd = originCloudInd(labels==i);
    % check percentage of pre and post point
    percentage = sum(originalInd)/clusterPoint.Count;
    if (percentage < settings.PERCENTAGE_THRESHOLD || percentage > (1-settings.PERCENTAGE_THRESHOLD))
        continue;
    end
    % remove outliers (clusters that fits a plane)
    referenceVector = [0,0,1];
    [~, inliers, ~, meanError] = pcfitplane(clusterPoint, settings.MIN_DIST_PLANEFIT,referenceVector);
    inlierPercentage = numel(inliers)/clusterPoint.Count;
    if (inlierPercentage > .5)
%         fprintf("exclude cluster #%d, its fit error:%d \n",i,mean(meanError));
        continue;
    end
    %     if (mean(meanError)) < .1
    %         continue;
    %     end
    numValidClusters = numValidClusters + 1;
    % write cluster number to the 'intensity' field
    clusterPoint.Intensity = repmat(numValidClusters,clusterPoint.Count,1);
    clusterPointCloudList=[clusterPointCloudList;clusterPoint];
    
end
assert(numel(clusterPointCloudList)>0);
% figure;
% clusterPointCloudForExport = pccat(clusterPointCloudList);
% pcshowpair(ptCloud,clusterPointCloudForExport);

%% calculate volume of each clusters, using alphashape
clusterVolumeList = zeros(numel(clusterPointCloudList),1);
for i=1:numel(clusterPointCloudList)
    xyzPoints = clusterPointCloudList(i).Location;
    shp = alphaShape(xyzPoints,2*settings.MIN_DIST_CLUSTER); % alpha radius should be tuned
    shp.RegionThreshold = 1; % only the first region
    clusterVolume = volume(shp);
    clusterVolumeList(i) = clusterVolume;
end

end

