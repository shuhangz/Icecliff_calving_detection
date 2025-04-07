function resultWithSnowVolume = calculateSnowVolume(result,snowDepth, settings)
%CALCULATESNOWVOLUME 计算冰雪体积及其误差
%   输入:
%   result - 输入的结构体
%   snowDepth - 雪深
%   settings - 设置参数
%   输出:
%   resultWithSnowVolume - 包含冰雪体积及误差的结构体

resultWithSnowVolume = result;

% 误差参数设置
sigma_alignment = 0.05; % 对齐误差
sigma_s = 0.0270/100; % 比例误差
sigma_depth = 0.28; % 雪深测量误差

for i = 1:length(resultWithSnowVolume)
    clusterPointCloudList = resultWithSnowVolume(i).collapsePointCloudClusters;
    snowVolumeList = zeros(numel(clusterPointCloudList),1);
    iceVolumeList = zeros(numel(clusterPointCloudList),1);
    clusterVolumeList = zeros(numel(clusterPointCloudList),1);
    snowVolumeErrorList = zeros(numel(clusterPointCloudList),1);
    iceVolumeErrorList = zeros(numel(clusterPointCloudList),1);
    clusterVolumeErrorList = zeros(numel(clusterPointCloudList),1);

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
        % 计算每个三角形的顶点坐标差向量
        v1 = xyz(triValid(:,2), :) - xyz(triValid(:,1), :);
        v2 = xyz(triValid(:,3), :) - xyz(triValid(:,2), :);

        % 计算叉积得到面积向量
        cp = cross(v1, v2);

        % 计算每个三角形的面积
        triangleAreas = 0.5 * sqrt(sum(cp.^2, 2));
        % TODO
        
        % 以三棱柱的方式计算雪体积，三角面片的面积乘以雪深
        snowVolumes = triangleAreas*snowDepth;
        snowVolume = sum(snowVolumes);
        surfaceArea = sum(triangleAreas);
        if snowVolume > clusterVolume % rare case
            %             disp('snow volume is larger than cluster volume') ;
            snowVolume = clusterVolume; % 这个cluster只包含了雪
        %                 % debug plot
        %     trisurf(tri,xyz(:,1),xyz(:,2),xyz(:,3), ...
        %     isValid*1.0,'FaceAlpha',0.8);
        %     axis equal
        % %                 hold on
        % %                 quiver3(P(:,1),P(:,2),P(:,3), ...
        % %                     fNormal(:,1),fNormal(:,2),fNormal(:,3),0.5,'color','r');
        % % end debug
        end
        
        sigma_snow = surfaceArea * sigma_depth;

        % 判断这个cluster是否只包含了雪
        a = clusterPoint.XLimits(2)-clusterPoint.XLimits(1);        
        b = clusterPoint.YLimits(2)-clusterPoint.YLimits(1);
        c = clusterPoint.ZLimits(2)-clusterPoint.ZLimits(1);
        assert(a>=0);
        assert(b>=0);
        assert(c>=0);
        clusterBoudingBoxDimensions = [a,b,c];
        % 计算cluster体积误差(基于边界框尺寸)
        sigma_a = sigma_alignment + sigma_s * a;
        sigma_b = sigma_alignment + sigma_s * b;
        sigma_c = sigma_alignment + sigma_s * c;
        boundingBoxVolume = a * b * c;
        sigma_boundingBoxVolume = sqrt(sigma_a^2 * b^2 * c^2 + a^2 * sigma_b^2 * c^2 + a^2 * b^2 * sigma_c^2);
        
        if any(clusterBoudingBoxDimensions < snowDepth) % this cluster contains only snow
            snowVolume = clusterVolume;            
            iceVolume = 0;
            sigma_ice = 0; % 当只包含雪时，冰的体积误差应为0
        else % 包含冰和雪
            iceVolume = clusterVolume - snowVolume;
            % 冰体积误差 (误差传递)
            sigma_ice = sqrt(sigma_snow^2 + sigma_boundingBoxVolume^2);
            if sigma_ice > iceVolume
                sigma_ice = iceVolume;
            end
        end       
        
        
        
        % 总体积误差 (基于边界框尺寸)
        sigma_total = sigma_boundingBoxVolume;
                
        snowVolumeList(j) = snowVolume;
        iceVolumeList(j) = iceVolume;
        snowVolumeErrorList(j) = sigma_snow;
        iceVolumeErrorList(j) = sigma_ice;
        clusterVolumeErrorList(j) = sigma_total;
    end
    
    resultWithSnowVolume(i).snowVolumeList = snowVolumeList;
    resultWithSnowVolume(i).iceVolumeList = iceVolumeList;
    resultWithSnowVolume(i).collapseVolumeList = clusterVolumeList;
    resultWithSnowVolume(i).snowVolumeErrorList = snowVolumeErrorList;
    resultWithSnowVolume(i).iceVolumeErrorList = iceVolumeErrorList;
    resultWithSnowVolume(i).collapseVolumeErrorList = clusterVolumeErrorList;
end


end

