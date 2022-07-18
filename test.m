clc;clear;
folder = 'D:\Working_Project\Point cloud\2022_haibaowan\cropped\';
fileName = '0130-31-mesh-sampled.las';
% fileName = '0130-31-cloud.las';
path = strcat(folder,fileName);
MIN_DIST_CLUSTER = .9;
MIN_DIST_PLANEFIT = .5;
PERCENTAGE_THRESHOLD = .1;

%% read point cloud
s = LASread(path,false,true);
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
% lasreader = lasFileReader(path);
% ptCloud = readPointCloud(lasreader);
pcshow(ptCloud);

%% Cluster point cloud based on distance
minDistance = MIN_DIST_CLUSTER;
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
LASwrite(s,strcat(folder,'cluster.las'),'version',14); 

%% keep clusters that contains both pre- and post- points
clusterPointClouds = pointCloud.empty
for i=1:numClusters
    clusterPoint = pointCloud(xyzPoints(labels==i,:), 'Normal', normals(labels==i,:));
    originalInd = originCloudInd(labels==i);
    % check percentage of pre and post point
    percentage = sum(originalInd)/clusterPoint.Count;
    if (percentage < PERCENTAGE_THRESHOLD || percentage > (1-PERCENTAGE_THRESHOLD))
        continue;
    end
    % remove outliers (clusters that fits a plane)
    referenceVector = [0,0,1];
    [~, inliers, ~, meanError] = pcfitplane(clusterPoint, MIN_DIST_PLANEFIT,referenceVector);
    inlierPercentage = numel(inliers)/clusterPoint.Count;
    if (inlierPercentage > .5)
        fprintf("exclude cluster #%d, its fit error:%d \n",i,mean(meanError));
        continue;
    end
%     if (mean(meanError)) < .1
%         continue;
%     end
    clusterPointClouds=[clusterPointClouds;clusterPoint];
    
end
assert(numel(clusterPointClouds)>0);
figure;
pcshowpair(ptCloud,clusterPointCloud);
%% calculate volume of each clusters, using alphashape
for i=1:numel(clusterPointClouds)
    xyzPoints = clusterPointClouds(i).Location;
    shp = alphaShape(xyzPoints,2*MIN_DIST_CLUSTER);
    shp.RegionThreshold = 1;
    clusterVolume = volume(shp)
    % debug
    pcshow(clusterPointClouds(i),'MarkerSize',10);
    hold on;
    plot(shp);
    axis equal;
    close all;
end

%% export clusters
clusterPointCloud = pccat(clusterPointClouds);
pcwrite(clusterPointCloud,strcat(folder,'diff.ply'),'Encoding','binary');


