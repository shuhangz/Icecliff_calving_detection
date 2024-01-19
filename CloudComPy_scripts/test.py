import os
import sys
import math
import psutil

# os.environ["_CCTRACE_"]="ON" # only if you want C++ debug traces

import cloudComPy as cc
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import colors
from scipy.optimize import curve_fit
from pylab import *

path = 'D:\\Working_Project\\Point cloud\\2022_haibaowan\\diff\\Haibaowan_land_only_mesh.bin'

STEP = 1
DISTANCE_THRESHOLD = .35
WRITE_RESULT = True
working_dir = os.path.dirname(path)

list_mean_front = []
list_var_front = []
list_mean_back = []
list_var_back = []


def main():
    write_path = os.path.join(working_dir,"distance_threshold_"+str(DISTANCE_THRESHOLD))
    res = cc.importFile(path)
    meshes = res[0]
    list_mean_front = np.zeros(len(meshes))
    list_var_front = np.zeros(len(meshes))
    list_mean_back = np.zeros(len(meshes))  
    list_var_back = np.zeros(len(meshes))

    for i in range(0, len(meshes)-1, STEP):
        # i=0
        if (i+STEP > len(meshes)-1):
            break
        first_epoch_mesh = meshes[i]
        next_epoch_mesh = meshes[i+STEP]
        first_epoch_date = first_epoch_mesh.getName()[4:8]
        next_epoch_date = next_epoch_mesh.getName()[4:8]
        changed_pointcloud,first_epoch_point,next_epoch_point = compare_mesh(
            first_epoch_mesh, next_epoch_mesh, DISTANCE_THRESHOLD)
        
        # compute mean and variance of front and back surface
        mean_front,var_front = histogram_statistics(first_epoch_point)
        mean_back,var_back = histogram_statistics(next_epoch_point)
        list_mean_front[i] = mean_front
        list_var_front[i] = var_front
        list_mean_back[i] = mean_back
        list_var_back[i] = var_back

        # histogram_fit(changed_pointcloud)
        pass

        if WRITE_RESULT:
            filename = first_epoch_date + '-' + next_epoch_date + '.las'
            
            # check if write_path exist, if not, create the folder
            if not os.path.exists(write_path):
                os.makedirs(write_path)
            # write changed point cloud
            ret = cc.SavePointCloud(
                changed_pointcloud, os.path.join(write_path, filename))

            if ret == cc.CC_FILE_ERROR.CC_FERR_NO_ERROR:
                print('successfully write point cloud:' +
                    os.path.join(write_path, filename))
            else:
                raise Exception("cannot write point cloud")

    # # plot mean and variance
    # plt.figure()
    # plt.plot(list_mean_front, label='front')
    # plt.plot(list_mean_back, label='back')
    # plt.legend()
    # plt.title('mean')
    # plt.figure()
    # plt.plot(list_var_front, label='front')
    # plt.plot(list_var_back, label='back')
    # plt.legend()
    # plt.title('variance')
    # plt.show()

    # export mean and variance to a single csv file
    np.savetxt(os.path.join(write_path,'mean_var.csv'),np.transpose([list_mean_front,list_var_front,list_mean_back,list_var_back]),delimiter=',',header='mean_front,var_front,mean_back,var_back',comments='')



    print('job done')


def compare_mesh(first_epoch_mesh, next_epoch_mesh, distance_threshold=0.3, sample_point_density=100):
    # sample point on each mesh
    first_epoch_point = first_epoch_mesh.samplePoints(densityBased=True, samplingParameter=sample_point_density,
                                                      withNormals=True)
    next_epoch_point = next_epoch_mesh.samplePoints(densityBased=True, samplingParameter=sample_point_density,
                                                    withNormals=True)
    # compute point to mesh distance
    nbCpu = psutil.cpu_count()
    bestOctreeLevel = cc.DistanceComputationTools.determineBestOctreeLevel(
        first_epoch_point, next_epoch_mesh)

    params = cc.Cloud2MeshDistancesComputationParams()
    params.maxThreadCount = nbCpu
    params.octreeLevel = bestOctreeLevel
    params.signedDistances = True

    cc.DistanceComputationTools.computeCloud2MeshDistances(
        first_epoch_point, next_epoch_mesh, params)
    cc.DistanceComputationTools.computeCloud2MeshDistances(
        next_epoch_point, first_epoch_mesh, params)

    front_surface_point = process_point_cloud(
        first_epoch_point, distance_threshold, True)
    back_surface_point = process_point_cloud(
        next_epoch_point, distance_threshold, False)

    changed_pointcloud = front_surface_point.cloneThis()
    changed_pointcloud.fuse(back_surface_point)
    changed_pointcloud.setCurrentDisplayedScalarField(5)

    # debug
    # print(changed_pointcloud.getScalarFieldDic())
    # ret = cc.SavePointCloud(front_surface_point, os.path.join(working_dir,'front.las'))
    # ret2 = cc.SavePointCloud(back_surface_point, os.path.join(working_dir,'back.las'))

    return changed_pointcloud,first_epoch_point,next_epoch_point


def process_point_cloud(point_cloud, distance_threshold, is_front=True):
    sf_dist_first = point_cloud.getScalarField(
        point_cloud.getScalarFieldDic()['C2M absolute distances'])
    point_cloud.setCurrentScalarField(
        point_cloud.getScalarFieldDic()['C2M absolute distances'])
    # front_surface_point = first_epoch_point.filterPointsByScalarValue(distance_threshold, sf_dist_first.getMax())
    if is_front:
        surface_point = point_cloud.filterPointsByScalarValue(
            distance_threshold, sf_dist_first.getMax())
    else:
        surface_point = point_cloud.filterPointsByScalarValue(
            sf_dist_first.getMin(), -distance_threshold)
    surface_point.exportNormalToSF(True, True, True)
    ind = surface_point.addScalarField("original_cloud_index")

    sf = surface_point.getScalarField(ind)
    if is_front:
        sf.fill(0)
        # sf.fromNpArrayCopy(np.zeros(sf.currentSize(),dtype=np.float32))
    else:
        sf.fill(1)
        # sf.fromNpArrayCopy(np.ones(sf.currentSize(),dtype=np.float32))

    return surface_point

def histogram_statistics(point_cloud):
    sf = point_cloud.getScalarField(point_cloud.getScalarFieldDic()[
                                    'C2M absolute distances'])
    mean,var = sf.computeMeanAndVariance()
    # asf = sf.toNpArray()
    # draw histogram of asf
    # (yhist, xhist) = np.histogram(asf, bins=256) # numpy histogram (without graphics)
    # plt.hist(asf,bins=256)
    

    return mean,var



def histogram_fit(point_cloud):
    sf = point_cloud.getScalarField(point_cloud.getScalarFieldDic()[
                                    'C2M absolute distances'])
    asf = sf.toNpArray()
    (yhist, xhist) = np.histogram(asf, bins=256) # numpy histogram (without graphics)
    xh = np.where(yhist > 0)[0]
    yh = yhist[xh]
    def gaussian(x, a, mean, sigma):
        return a * np.exp(-((x - mean)**2 / (2 * sigma**2)))

    popt, pcov = curve_fit(gaussian, xh, yh, [10000, 100, 10])

    plt.plot(yhist)    
    i = np.linspace(0, 300, 301)
    plt.plot(i, gaussian(i, *popt))

if __name__ == '__main__':
    main()
