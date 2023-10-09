clc;clear; close all;
addpath('.\function');
addpath('.\function\3rdparty');
% folder = 'D:\Working_Project\Point cloud\2022_haibaowan\diff\';
folder = 'D:\Working_Project\Point cloud\2022_haibaowan\diff\distance_threshold_0.348';

dirLASFile = dir(fullfile(folder,'*.las'));
% dirLASFile = dir(fullfile(folder,'0216-0217.las'));

outputSubFolder = 'export_single';
validSpaceFilename = 'pointcloudValidSpace.mat';

% mkdir(fullfile(folder,outputSubFolder));

SNOW_DEPTH = 0.66;
YEAR = '2022';

settings = struct;
settings.FILTER_METHOD_USED = [1,1,1,1,1];
settings.MIN_DIST_CLUSTER = .9;
settings.MIN_DIST_PLANEFIT = .1;
settings.PERCENTAGE_THRESHOLD = .25;
settings.MAX_XYPLANE_AREA = 800;
settings.VOLUME_THRESHOLD = 2;
settings.NORMAL_ANGLE_THRESHOLD = 45;

settings.DEBUG = false;
settings.EXPORT_MATRIX = true;
settings.EXPORT_POINTCLOUD = true;

result = struct('dateBefore',[],'dateAfter',[], 'collapsePointCloudClusters',[],...
    'collapseVolumeList',[],'snowVolumeList',[]);
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
    clusterPointCloudForExport = pccat(clusterPointCloudList);
    if settings.EXPORT_POINTCLOUD
    %     LASwrite(s,fullfile(folder,outputSubFolder,strcat(saveFilePrefix,'-cluster.las')),'version',14);
        pcwrite(clusterPointCloudForExport,fullfile(folder,outputSubFolder,strcat(saveFilePrefix,'_diff.ply')),'Encoding','binary');
    end

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
resultWithSnowVolume = calculateSnowVolume(result,SNOW_DEPTH,settings);

%% export result
v = zeros(length(resultWithSnowVolume),1);
vs = zeros(length(resultWithSnowVolume),1);
numCalving = zeros(length(resultWithSnowVolume),1);
db = string(zeros(length(resultWithSnowVolume),1));
da = string(zeros(length(resultWithSnowVolume),1));

for i=1:length(resultWithSnowVolume)
    v(i) = sum(resultWithSnowVolume(i).collapseVolumeList);
    vs(i) = sum(resultWithSnowVolume(i).snowVolumeList);
    numCalving(i) = numel(resultWithSnowVolume(i).collapsePointCloudClusters);

    db_MM = resultWithSnowVolume(i).dateBefore(1:2);
    db_dd = resultWithSnowVolume(i).dateBefore(3:4);
    da_MM = resultWithSnowVolume(i).dateAfter(1:2);
    da_dd = resultWithSnowVolume(i).dateAfter(3:4);
    db(i) = strcat(YEAR,'-',db_MM,'-',db_dd);
    da(i) = strcat(YEAR,'-',da_MM,'-',da_dd);    
    
    str = sprintf('%s相比%s的崩解体积为：%f 立方米，积雪体积：%f立方米',resultWithSnowVolume(i).dateAfter,...
        resultWithSnowVolume(i).dateBefore, v(i),vs(i));
    disp(str);
end

%% export result to xlsx
exportStruct = struct('dateBefore',[],'dateAfter',[], 'numCalve',[],'calvingVolume',[],'snowVolume',[]);
exportStruct.dateBefore = datetime(db,"InputFormat","yyyy-MM-dd");
exportStruct.dateAfter = datetime(da,"InputFormat","yyyy-MM-dd");
exportStruct.numCalve = numCalving;
exportStruct.calvingVolume = v;
exportStruct.snowVolume = vs;

writetable(struct2table(exportStruct), fullfile(folder,outputSubFolder,strcat('snow_volume_',string(SNOW_DEPTH),'.xlsx')))

%% export result as matrix
if settings.EXPORT_MATRIX
    dateList = unique([exportStruct.dateBefore;exportStruct.dateAfter]);
    numEpoch = length(dateList);
    dateDict = containers.Map(cellstr(datestr(dateList)),1:numEpoch);
    exportMatrix = zeros(numEpoch);
    for i=1:length(exportStruct.dateBefore)
        dateBefore = datestr(exportStruct.dateBefore(i));
        pos_before = dateDict(dateBefore);
        dateAfter = datestr(exportStruct.dateAfter(i));
        pos_after = dateDict(dateAfter);
        volume = exportStruct.calvingVolume(i);
        exportMatrix(pos_before,pos_after) = volume;
    end
end




