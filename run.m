clc;clear; close all;
addpath('.\function');
addpath('.\function\3rdparty');
% folder = 'D:\Working_Project\Point cloud\2022_haibaowan\diff\';
folder = 'D:\Working_Project\Point cloud\2022_haibaowan\diff\distance_threshold_0.35';

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
settings.NORMAL_ANGLE_THRESHOLD = 80;

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


%% settings
DENSITY_SNOW = 440; % snow density kg/m^3
DENSITY_ICE = 917; % ice density kg/m^3
SIGMA_SNOW_DENSITY = 50; % std of snow density

%% calculate snow volume
resultWithSnowVolume = calculateSnowVolume(result,SNOW_DEPTH,settings);

%% export result
v = zeros(length(resultWithSnowVolume),1);
vs = zeros(length(resultWithSnowVolume),1);
vi = zeros(length(resultWithSnowVolume),1);
v_err = zeros(length(resultWithSnowVolume),1);
vs_err = zeros(length(resultWithSnowVolume),1);
vi_err = zeros(length(resultWithSnowVolume),1);
ms = zeros(length(resultWithSnowVolume),1);
mi = zeros(length(resultWithSnowVolume),1);
ms_err = zeros(length(resultWithSnowVolume),1);
mi_err = zeros(length(resultWithSnowVolume),1);
numCalving = zeros(length(resultWithSnowVolume),1);
db = string(zeros(length(resultWithSnowVolume),1));
da = string(zeros(length(resultWithSnowVolume),1));

for i=1:length(resultWithSnowVolume)
    v(i) = sum(resultWithSnowVolume(i).collapseVolumeList);
    vs(i) = sum(resultWithSnowVolume(i).snowVolumeList);
    vi(i) = sum(resultWithSnowVolume(i).iceVolumeList);
    v_err(i) = sum(resultWithSnowVolume(i).collapseVolumeErrorList);
    vs_err(i) = sum(resultWithSnowVolume(i).snowVolumeErrorList);
    vi_err(i) = sum(resultWithSnowVolume(i).iceVolumeErrorList);
    
    % calculate mass and mass error
    ms(i) = vs(i) * DENSITY_SNOW;
    mi(i) = vi(i) * DENSITY_ICE;
    ms_err(i) = sqrt((vs(i)*SIGMA_SNOW_DENSITY)^2 + (DENSITY_SNOW*vs_err(i))^2);
    mi_err(i) = DENSITY_ICE * vi_err(i);
    
    numCalving(i) = numel(resultWithSnowVolume(i).collapsePointCloudClusters);

    db_MM = resultWithSnowVolume(i).dateBefore(1:2);
    db_dd = resultWithSnowVolume(i).dateBefore(3:4);
    da_MM = resultWithSnowVolume(i).dateAfter(1:2);
    da_dd = resultWithSnowVolume(i).dateAfter(3:4);
    db(i) = strcat(YEAR,'-',db_MM,'-',db_dd);
    da(i) = strcat(YEAR,'-',da_MM,'-',da_dd);    
    
    str = sprintf('%s相比%s的崩解体积为：%.2f±%.2f立方米(±%.1f%%)，积雪体积：%.2f±%.2f立方米(±%.1f%%)，冰体积：%.2f±%.2f立方米(±%.1f%%)\n积雪质量：%.1f±%.1f吨(±%.1f%%)，冰质量：%.1f±%.1f吨(±%.1f%%)',...
        resultWithSnowVolume(i).dateAfter, resultWithSnowVolume(i).dateBefore, ...
        v(i), v_err(i), (v_err(i)/v(i))*100, vs(i), vs_err(i), (vs_err(i)/vs(i))*100, vi(i), vi_err(i), (vi_err(i)/vi(i))*100, ...
        ms(i)/1000, ms_err(i)/1000, (ms_err(i)/ms(i))*100, mi(i)/1000, mi_err(i)/1000, (mi_err(i)/mi(i))*100);
    disp(str);
end

%% export result to xlsx
exportStruct = struct('dateBefore',[],'dateAfter',[], 'numCalve',[],...
    'calvingVolume',[],'calvingVolumeError',[],'calvingVolumeErrorPercentage',[],...
    'snowVolume',[],'snowVolumeError',[],'snowVolumeErrorPercentage',[],...
    'iceVolume',[],'iceVolumeError',[],'iceVolumeErrorPercentage',[],...
    'snowMass',[],'snowMassError',[],'snowMassErrorPercentage',[],...
    'iceMass',[],'iceMassError',[],'iceMassErrorPercentage',[]);
exportStruct.dateBefore = datetime(db,"InputFormat","yyyy-MM-dd");
exportStruct.dateAfter = datetime(da,"InputFormat","yyyy-MM-dd");
exportStruct.numCalve = numCalving;
exportStruct.calvingVolume = v;
exportStruct.calvingVolumeError = v_err;
exportStruct.calvingVolumeErrorPercentage = zeros(size(v));
nonZeroV = v ~= 0;
exportStruct.calvingVolumeErrorPercentage(nonZeroV) = (v_err(nonZeroV)./v(nonZeroV))*100;

exportStruct.snowVolume = vs;
exportStruct.snowVolumeError = vs_err;
exportStruct.snowVolumeErrorPercentage = zeros(size(vs));
nonZeroVs = vs ~= 0;
exportStruct.snowVolumeErrorPercentage(nonZeroVs) = (vs_err(nonZeroVs)./vs(nonZeroVs))*100;

exportStruct.iceVolume = vi;
exportStruct.iceVolumeError = vi_err;
exportStruct.iceVolumeErrorPercentage = zeros(size(vi));
nonZeroVi = vi ~= 0;
exportStruct.iceVolumeErrorPercentage(nonZeroVi) = (vi_err(nonZeroVi)./vi(nonZeroVi))*100;

% 将质量单位从kg转换为吨
exportStruct.snowMass = ms/1000;
exportStruct.snowMassError = ms_err/1000;
exportStruct.snowMassErrorPercentage = zeros(size(ms));
nonZeroMs = ms ~= 0;
exportStruct.snowMassErrorPercentage(nonZeroMs) = (ms_err(nonZeroMs)./ms(nonZeroMs))*100;

exportStruct.iceMass = mi/1000;
exportStruct.iceMassError = mi_err/1000;
exportStruct.iceMassErrorPercentage = zeros(size(mi));
nonZeroMi = mi ~= 0;
exportStruct.iceMassErrorPercentage(nonZeroMi) = (mi_err(nonZeroMi)./mi(nonZeroMi))*100;

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




