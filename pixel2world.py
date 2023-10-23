import numpy as np
import torch
import math
import open3d as o3d
def to_lnglat(xy, w, h):
    '''
    xy = np.array([(0, 0), (1, 1)])
    '''

    x = xy[:,0]
    y = xy[:,1]
    lng = x / (w / 360) - 180
    lat = y / (h / 180)
    lng = 360 - ((lng + 360) % 360)
    #print(np.array((lng, lat)).T)
    return np.deg2rad(np.array((lng, lat)).T)


def to_cartesian(lnglat):
    '''
    r = 1
    '''
    # center of image will be (1, 0, 0)
    lng = lnglat[:,0] # 2pi
    lat = lnglat[:,1] # pi/2
    x = np.sin(lat)*np.cos(lng) 
    y = np.sin(lng)*np.sin(lat)
    z = np.cos(lat)
    # center of image will be (0, 1, 0)
    tmp = x
    x = -y
    y = tmp
    return np.array((x, y, z)).T



#result = to_lnglat(np.array([(0, 0), (512, -256)]), 1024 / (2*math.pi))
def pixel2world(w, h):
    '''
    cam_pos = np.array([(x, y, z)])
    w = 1024
    h = 512
    '''
    points = np.array([(i, j) for j in range(h) for i in range(w)])
    lnglat = to_lnglat(points, w, h)
    result = to_cartesian(lnglat)
    #result = result.reshape((h, w, 3))
    #result = np.expand_dims(result, axis=0) # (m, h, w, 3)
    print('ray direction shape: ', result.shape)
    # return ray directions of each pixel.
    return result




#pixel2world((1, 0, 0), 1024, 512)

