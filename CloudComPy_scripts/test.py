import os
import sys
import math
import psutil

# os.environ["_CCTRACE_"]="ON" # only if you want C++ debug traces

import cloudComPy as cc
import numpy as np
path = 'D:\\Working_Project\\Point cloud\\2022_haibaowan\\diff\\Haibaowan_land_only_mesh.bin'
working_dir = os.path.dirname(path)
def main():
    res = cc.importFile(path)
    meshes = res[0]

    for i in range(0,len(meshes)-1): 
        # i=0
        first_epoch_mesh = meshes[i]
        next_epoch_mesh = meshes[i+1]
        first_epoch_date = first_epoch_mesh.getName()[4:8]
        next_epoch_date = next_epoch_mesh.getName()[4:8]
        changed_pointcloud = compare_mesh(first_epoch_mesh,next_epoch_mesh)

        filename = first_epoch_date + '-' + next_epoch_date + '.las'
        ret = cc.SavePointCloud(changed_pointcloud, os.path.join(working_dir,filename))

        if ret==cc.CC_FILE_ERROR.CC_FERR_NO_ERROR:
            print('successfully write point cloud:'+ os.path.join(working_dir,filename))
        else:
            raise Exception("cannot write point cloud")
    
    print('job done')

def compare_mesh(first_epoch_mesh, next_epoch_mesh, distance_threshold=0.3, sample_point_density=100):
    # sample point on each mesh
    first_epoch_point = first_epoch_mesh.samplePoints(densityBased=True,samplingParameter=sample_point_density,
    withNormals=True)
    next_epoch_point = next_epoch_mesh.samplePoints(densityBased=True,samplingParameter=sample_point_density,
    withNormals=True)
    # compute point to mesh distance 
    nbCpu = psutil.cpu_count()
    bestOctreeLevel = cc.DistanceComputationTools.determineBestOctreeLevel(first_epoch_point,next_epoch_mesh)

    params = cc.Cloud2MeshDistancesComputationParams()
    params.maxThreadCount = nbCpu
    params.octreeLevel = bestOctreeLevel
    params.signedDistances = True

    cc.DistanceComputationTools.computeCloud2MeshDistances(first_epoch_point, next_epoch_mesh, params)
    cc.DistanceComputationTools.computeCloud2MeshDistances(next_epoch_point, first_epoch_mesh, params)

    front_surface_point = process_point_cloud(first_epoch_point,distance_threshold,True)
    back_surface_point = process_point_cloud(next_epoch_point,distance_threshold,False)

    changed_pointcloud = front_surface_point.cloneThis()
    changed_pointcloud.fuse(back_surface_point)
    changed_pointcloud.setCurrentDisplayedScalarField(5)
    
    # debug
    # print(changed_pointcloud.getScalarFieldDic())
    # ret = cc.SavePointCloud(front_surface_point, os.path.join(working_dir,'front.las'))
    # ret2 = cc.SavePointCloud(back_surface_point, os.path.join(working_dir,'back.las'))

    return changed_pointcloud

def process_point_cloud(first_epoch_point,distance_threshold,is_front=True):
    sf_dist_first = first_epoch_point.getScalarField(first_epoch_point.getScalarFieldDic()['C2M absolute distances'])
    first_epoch_point.setCurrentScalarField(first_epoch_point.getScalarFieldDic()['C2M absolute distances'])
    # front_surface_point = first_epoch_point.filterPointsByScalarValue(distance_threshold, sf_dist_first.getMax())
    if is_front:        
        front_surface_point = first_epoch_point.filterPointsByScalarValue(distance_threshold, sf_dist_first.getMax())
    else:
        front_surface_point = first_epoch_point.filterPointsByScalarValue(sf_dist_first.getMin(), -distance_threshold)
    front_surface_point.exportNormalToSF(True,True,True)
    ind = front_surface_point.addScalarField("original_cloud_index")

    sf = front_surface_point.getScalarField(ind)
    if is_front:
        sf.fill(0)
        # sf.fromNpArrayCopy(np.zeros(sf.currentSize(),dtype=np.float32))
    else:
        sf.fill(1)
        # sf.fromNpArrayCopy(np.ones(sf.currentSize(),dtype=np.float32))
    
    return front_surface_point





if __name__ == '__main__':
    main()