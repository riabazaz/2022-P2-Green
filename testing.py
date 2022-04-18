import numpy as np
from numpy import linspace,zeros,pi,rad2deg, append, round, maximum, deg2rad, reshape, load, meshgrid, c_, ones
from scipy.interpolate import interp1d, griddata 
from math import floor

def createGrid(x_spacing, y_spacing):
    # create the array for the x points
    x_lin = [0]
    while x_lin[-1]+x_spacing < 20: # 20cm is roughly 8inch
        x_lin.append(x_lin[-1]+x_spacing) # *0.394 
    
    # create the array for the y points
    y_lin = [0]
    while y_lin[-1]+y_spacing < 20: # 27cm is roughly 11inch 
        y_lin.append(y_lin[-1]+y_spacing) 

    nx, ny = (len(x_lin),len(y_lin))     #can be adjusted to add more calibration points
    xv,yv = meshgrid(x_lin,y_lin,indexing='xy')
    grid = c_[xv.reshape(nx*ny,1), yv.reshape(nx*ny,1), zeros((nx*ny,1)), ones((nx*ny,1))]
    grid_idx = list(range(nx*ny))
    for i in range(nx-(nx % 2)):
        idx = nx*(2*i+1)
        grid_idx[idx:idx+nx] = grid_idx[idx:idx+nx][::-1]
    return grid, nx, ny #dot(grid,self.Tp2w.T),


def load_cal_ang():
    calib_ang_b = load("calib_ang_b.npy")
    calib_ang_a = load("calib_ang_a.npy")
    calib_ang_s = load("calib_ang_s.npy")
    return (calib_ang_b, calib_ang_a, calib_ang_s)

def interpolateLocation(x, y, calib_ang_b, calib_ang_a, calib_ang_s, points):

    angA = np.reshape(calib_ang_a,(9,1))
    angA = np.divide(angA, 100)
    griddatapoints = points[...,:-2]

    print(angA)
    print(griddatapoints)


        
    aa = griddata(griddatapoints, angA,(x,y))
    ba = griddata(griddatapoints, calib_ang_b,(x,y))
    sa = griddata(griddatapoints, calib_ang_s,(x,y))
        
    return(ba,aa,sa)

if __name__=="__main__": 
    calib_ang_b, calib_ang_a, calib_ang_s = load_cal_ang()
    grid, nx, ny = createGrid(8,8)
    interpolateLocation(4,5,calib_ang_b, calib_ang_a, calib_ang_s,grid)
