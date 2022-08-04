clc;clear; close all;
addpath('.\function');
addpath('.\function\3rdparty');
folder = 'D:\Working_Project\Point cloud\2022_haibaowan\diff\';
dirLASFile = dir(fullfile(folder,'*.las'));
outputSubFolder = 'export';
validSpaceFilename = 'pointcloudValidSpace.mat';

mkdir(fullfile(folder,outputSubFolder));

settings = struct;
settings.FILTER_METHOD_USED = [1,1,1,1,1];
settings.MIN_DIST_CLUSTER = .9;
settings.MIN_DIST_PLANEFIT = .1;
settings.PERCENTAGE_THRESHOLD = .25;
settings.MAX_XYPLANE_AREA = 800;
settings.VOLUME_THRESHOLD = 2;
settings.NORMAL_ANGLE_THRESHOLD = 45;

settings.DEBUG = true;

result = struct('dateBefore',{},'dateAfter',{}, 'collapsePointCloudClusters',{},...
    'collapseVolumeList',{},'snowVolumeList',{});
load(fullfile(folder,validSpaceFilename));

for i=1:length(dirLASFile)
    lasPath = fullfile(dirLASFile(i).folder, dirLASFile(i).name);
    % read point cloud
    s = LASread(lasPath,false,true);
    [clusterPointCloudList,~] = detectChange(s,settings,pointcloudValidSpace);

    result(i).dateBefore = dirLASFile(i).name(1:4);
    result(i).dateAfter = dirLASFile(i).name(6:9);
    result(i).collapsePointCloudClusters = clusterPointCloudList;
    
    % write clustered LAS file
    saveFilePrefix = strcat(result(i).dateBefore,'-',result(i).dateAfter);
    %     LASwrite(s,fullfile(folder,outputSubFolder,strcat(saveFilePrefix,'-cluster.las')),'version',14);
    clusterPointCloudForExport = pccat(clusterPointCloudList);
    pcwrite(clusterPointCloudForExport,fullfile(folder,outputSubFolder,strcat(saveFilePrefix,'_diff.ply')),'Encoding','binary');
    
    if settings.DEBUG
        xyzPoints = [s.record.x s.record.y s.record.z];
        try
            normals = [s.record.normalx s.record.normaly s.record.normalz];
        catch
            normals = [s.record.normal_x s.record.normal_y s.record.normal_z];
        end
        if isempty(normals)
            ptCloud = pointCloud(xyzPoints);
        else
            ptCloud = pointCloud(xyzPoints,'Normal',normals);
        end
        figure('Name',saveFilePrefix);
        clusterPointCloudForExport = pccat(clusterPointCloudList);
        pcshowpair(ptCloud,clusterPointCloudForExport);
    end
    
end


%% calculate snow volume
SNOW_DEPTH = 0.3;
resultWithSnowVolume = calculateSnowVolume(result,SNOW_DEPTH,settings);


for i=1:length(dirLASFile)
    v = sum(resultWithSnowVolume(i).collapseVolumeList);
    str = sprintf('%s相比%s的崩解体积为：%f 立方米',resultWithSnowVolume(i).dateAfter, resultWithSnowVolume(i).dateBefore, v);
    disp(str);
end

