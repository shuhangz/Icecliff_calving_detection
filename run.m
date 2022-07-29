clc;clear;
addpath('.\function');
addpath('.\function\3rdparty');
folder = 'D:\Working_Project\Point cloud\2022_haibaowan\diff\';
dirLASFile = dir(fullfile(folder,'*.las'));
outputSubFolder = 'export';
mkdir(fullfile(folder,outputSubFolder));

settings = struct;
settings.MIN_DIST_CLUSTER = .9;
settings.MIN_DIST_PLANEFIT = .5;
settings.PERCENTAGE_THRESHOLD = .1;

result = struct('dateBefore',{},'dateAfter',{}, 'collapsePointCloudClusters',{},...
    'collapseVolumeList',{});
for i=1:length(dirLASFile)
    lasPath = fullfile(dirLASFile(i).folder, dirLASFile(i).name);
    % read point cloud
    s = LASread(lasPath,false,true);
    [clusterVolumeList,clusterPointCloudList,~] = calculateVolume(s,settings);
    
    result(i).dateBefore = dirLASFile(i).name(1:4);
    result(i).dateAfter = dirLASFile(i).name(6:9);
    result(i).collapsePointCloudClusters = clusterPointCloudList;
    result(i).collapseVolumeList = clusterVolumeList;   
    
    % write clustered LAS file
    saveFilePrefix = strcat(result(i).dateBefore,'-',result(i).dateAfter);
%     LASwrite(s,fullfile(folder,outputSubFolder,strcat(saveFilePrefix,'-cluster.las')),'version',14);
    clusterPointCloudForExport = pccat(clusterPointCloudList);
    pcwrite(clusterPointCloudForExport,fullfile(folder,outputSubFolder,strcat(saveFilePrefix,'_diff.ply')),'Encoding','binary');
    

end

for i=1:length(dirLASFile)
    v = sum(result(i).collapseVolumeList);
    str = sprintf('%s相比%s的崩解体积为：%f 立方米',result(i).dateAfter, result(i).dateBefore, v);
    disp(str);
end

