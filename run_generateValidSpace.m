clc;clear;close all;
addpath('.\function');
addpath('.\function\3rdparty');
folder = 'D:\Working_Project\Point cloud\2022_haibaowan\publish\distance_threshold_0.35';
dirLASFile = dir(fullfile(folder,'*.las'));
outputSubFolder = 'export';
mkdir(fullfile(folder,outputSubFolder));

settings = struct;
settings.FILTER_METHOD_USED = [1,0,0,0,0];
settings.MIN_DIST_CLUSTER = .9;
settings.MIN_DIST_PLANEFIT = .1;
settings.PERCENTAGE_THRESHOLD = .3;
settings.MAX_XYPLANE_AREA = 10000;
settings.VOLUME_THRESHOLD = 2;
settings.DEBUG = false;


result = struct('dateBefore',{},'dateAfter',{}, 'collapsePointCloudClusters',{},...
    'collapseVolumeList',{},'snowVolumeList',{});

i=1;
lasPath = fullfile(dirLASFile(i).folder, dirLASFile(i).name);
% read point cloud
s = LASread(lasPath,false,true);
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

%% clusters validity check
clusterPointCloudList = pointCloud.empty;
numValidClusters = 0;
for i=1:numClusters
    clusterPoint = pointCloud(xyzPoints(labels==i,:), 'Normal', normals(labels==i,:));
    originalInd = originCloudInd(labels==i);
    
    % check percentage of pre and post point,keep clusters that contains
    % both pre- and post- points
    % originalInd:(0--front--before, 1--back--after)
    percentage = sum(originalInd)/clusterPoint.Count;
    if (percentage < settings.PERCENTAGE_THRESHOLD || percentage > (1-settings.PERCENTAGE_THRESHOLD))
        continue;
    end
    
    
    % exclude tiny clusters
    shp = alphaShape(clusterPoint.Location,2*settings.MIN_DIST_CLUSTER); % alpha radius should be tuned
    clusterVolume = volume(shp);
    if (clusterVolume < settings.VOLUME_THRESHOLD)
        continue;
    end
    
    numValidClusters = numValidClusters + 1;
    % write cluster number to the 'intensity' field
    clusterPoint.Intensity = repmat(numValidClusters,clusterPoint.Count,1);
    clusterPointCloudList=[clusterPointCloudList;clusterPoint];
    
end

%% write clustered LAS file
% saveFilePrefix = strcat(dirLASFile(1).name(1:4),'-',dirLASFile(1).name(6:9));
% LASwrite(s,fullfile(folder,outputSubFolder,strcat(saveFilePrefix,'-cluster.las')),'version',14);
assert(numel(clusterPointCloudList)>0);
clusterPointCloudForExport = pccat(clusterPointCloudList);
% pcwrite(clusterPointCloudForExport,fullfile(folder,outputSubFolder,strcat(saveFilePrefix,'_diff.ply')),'Encoding','binary');
% if settings.DEBUG
%     pcshowpair(ptCloud,clusterPointCloudForExport);
% end
pointcloudValidSpace = clusterPointCloudForExport;
save(fullfile(folder,outputSubFolder,'pointcloudValidSpace.mat'), 'pointcloudValidSpace');
