from shapely.geometry import Point, LineString, MultiPoint
import numpy as np
from math import cos,sin,atan2,pi
import time
import matplotlib.pyplot as plt

def get_point_line(p1,p2,density,increase=0):
    x1 = p1.x
    y1 = p1.y
    x2 = p2.x
    y2 = p2.y

    a = np.array([x1,y1])
    b = np.array([x2,y2])

    vec = (b-a).T

    line = get_equidistant_points(a,b,round(np.linalg.norm(vec)/density)+1)

    if increase>0:
        line = increase_line(line,b-a,increase,density)

    return vec,line

def increase_line(line,vec,n,density):
    vec = vec/np.linalg.norm(vec)
    increase_vec = density * np.arange(1,n+1)
    increase_vec = increase_vec.reshape((len(increase_vec),1))
    increase_vec = increase_vec * vec
    before = line[0] * np.ones((n,2)) - increase_vec
    after = line[-1] * np.ones((n,2)) + increase_vec

    line = np.concatenate((before,line,after),axis=0)
    return line

def get_equidistant_points(p1, p2, n):
    return np.concatenate((np.expand_dims(np.linspace(p1[0], p2[0], max(n,2)), axis=1), np.expand_dims(np.linspace(p1[1], p2[1], max(n,2)), axis=1)),axis=1)

def points_arr_to_point_line(points,density):
    """ Input: array of (shapely) Points, density
        Output: array of points (shapely) Points where the distance between neighboring points is density. """
    point_line = np.zeros((1,2))
    for i in range(len(points)-1):
        p1 = points[i]
        p2 = points[i+1]

        _,line = get_point_line(p1,p2,density)
        point_line = np.append(point_line,line,axis=0)
    point_line = point_line[1:,:]
    return list(map(Point, zip(point_line[:,0], point_line[:,1])))


def points_to_graph_points(point1, point2, density=1, width=10, increase = 0):    # width equals threshold
    
    vec,point_line = get_point_line(point1,point2,density,increase)

    normal_vec = np.matmul(np.array([[cos(pi/2),-sin(pi/2)],[sin(pi/2),cos(pi/2)]]), vec)/np.linalg.norm(vec)

    num_points = round(width/density/2) + increase    # separately in positive and negative direction
    points_in_line = point_line.shape[0]

    all_points = np.zeros((points_in_line*(num_points*2+1),2))

    start_index = points_in_line*num_points + increase      # index of the original point 1 (start)
    goal_index = points_in_line*(num_points+1)-1 - increase # index of the original point 2 (goal)

    line_start_index = points_in_line*num_points       
    line_goal_index = points_in_line*(num_points+1)-1  

    all_points[line_start_index:line_goal_index+1,:] = point_line

    for i in range(num_points):
        pos_line_points = point_line + normal_vec*density*(i+1)
        neg_line_points = point_line - normal_vec*density*(i+1)
        all_points[points_in_line*(num_points+i+1):points_in_line*(num_points+i+2),:] = pos_line_points
        all_points[points_in_line*(num_points-i-1):points_in_line*(num_points-i),:] = neg_line_points


    """ if increase:
        plt.scatter(all_points[:,0], all_points[:,1])
        plt.savefig("{}.pdf".format(time.time())) """

    #all_points = list(map(Point, zip(all_points[:,0], all_points[:,1])))
    all_points = MultiPoint(all_points)
    point_line = MultiPoint(point_line)

    return all_points,point_line,start_index,goal_index