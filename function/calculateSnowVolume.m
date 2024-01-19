function resultWithSnowVolume = calculateSnowVolume(result,snowDepth, settings)
%CALCULATESNOWVOLUME 此处显示有关此函数的摘要
%   此处显示详细说明
resultWithSnowVolume = result;
% resultWithSnowVolume.snowVolumeList = {};

for i = 1:length(resultWithSnowVolume)
    clusterPointCloudList = resultWithSnowVolume(i).collapsePointCloudClusters;
    snowVolumeList = zeros(numel(clusterPointCloudList),1);

    clusterVolumeList = zeros(numel(clusterPointCloudList),1);

    for j=1:length(clusterPointCloudList)
        clusterPoint = clusterPointCloudList(j);
        xyzPoints = clusterPoint.Location;
        % calculate volume of each clusters, using alphashape
        shp = alphaShape(xyzPoints); % alpha radius should be tuned
        %         shp = alphaShape(xyzPoints); % alpha radius should be tuned
        pc  = criticalAlpha(shp,'one-region');
        shp.Alpha = pc;

        clusterVolume = volume(shp);
        clusterVolumeList(j) = clusterVolume;

        % calculate snow volume
        [tri, xyz] = boundaryFacets(shp);
        if (size(tri,2)) < 3
            error('wrong surface');
        end
        TR = triangulation(tri,xyz);
        fNormal = faceNormal(TR);
        P = incenter(TR); % center of each triangle
        vectorUp = repmat([0 0 1],length(fNormal),1);
        vC=cross(vectorUp,fNormal,2); %vectorized
        vNC=vecnorm(vC,2,2); % since only z-rotation is allowed anyway, this is equivalent to: vNC=vC(:,3)
        vD=dot(vectorUp,fNormal,2);
        vZenithAngleInDegrees = atan2d(vNC,vD); % angle between normal and up direction

        % using triangle ray intersection to determine upper surface that
        % has snow
        Z = ones(length(P),1)*clusterPoint.ZLimits(2)+(clusterPoint.ZLimits(2)-clusterPoint.ZLimits(1));

        rayOrigin = [P(:,1), P(:,2), Z];
        rayDirection = repmat([0 0 -1], length(P) ,1);
        distances = vecnorm((rayOrigin-P)')';

        vert1 = xyz(tri(:,1),:);
        vert2 = xyz(tri(:,2),:);
        vert3 = xyz(tri(:,3),:);
        [intersect,distanceHit,~,~,~] = TriangleRayIntersection(rayOrigin, ...
            rayDirection, vert1, vert2, vert3,'planeType','one sided');

        isOcclusionFree = (distances<=distanceHit+0.05);
        isInValidZnd = vZenithAngleInDegrees<settings.NORMAL_ANGLE_THRESHOLD;
        isValid = (isOcclusionFree & isInValidZnd & intersect);
        triValid = tri(isValid,:);
        v1 = xyz(triValid(:,2), :) - xyz(triValid(:,1), :);
        v2 = xyz(triValid(:,3), :) - xyz(triValid(:,2), :);
        cp = 0.5*cross(v1,v2);
        surfaceArea = sum(sqrt(dot(cp, cp, 2))); 

        snowVolume = surfaceArea*snowDepth;
        if snowVolume > clusterVolume
            snowVolume = clusterVolume;
        %                 % debug plot
        %     trisurf(tri,xyz(:,1),xyz(:,2),xyz(:,3), ...
        %     isValid*1.0,'FaceAlpha',0.8);
        %     axis equal
        % %                 hold on
        % %                 quiver3(P(:,1),P(:,2),P(:,3), ...
        % %                     fNormal(:,1),fNormal(:,2),fNormal(:,3),0.5,'color','r');
        % % end debug
        end
        %add by zhz
        a = clusterPoint.XLimits(2)-clusterPoint.XLimits(1);        
        b = clusterPoint.YLimits(2)-clusterPoint.YLimits(1);
        c = clusterPoint.ZLimits(2)-clusterPoint.ZLimits(1);
        clusterBoudingBoxDimensions = [a,b,c];
        % boundingBox = [clusterPoint.XLimits, clusterPoint.YLimits, clusterPoint.ZLimits];
        if any(clusterBoudingBoxDimensions < snowDepth) 
            snowVolume = clusterVolume;

        end
        %add by zhz
        snowVolumeList(j) = snowVolume;
    end
    resultWithSnowVolume(i).snowVolumeList = snowVolumeList;
    resultWithSnowVolume(i).collapseVolumeList = clusterVolumeList;
end


end

