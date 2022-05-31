from shapely.geometry import Point, LineString, MultiPoint
import numpy as np
from math import cos,sin,atan2,pi
import matplotlib.pyplot as plt

def getEquidistantPoints(p1, p2, n):
    return np.concatenate((np.expand_dims(np.linspace(p1[0], p2[0], n), axis=1), np.expand_dims(np.linspace(p1[1], p2[1], n), axis=1)),axis=1)

def points_to_graph_points(point1, point2, density=1, width=10):    # width equals threshold
    x1 = point1.x
    y1 = point1.y
    x2 = point2.x
    y2 = point2.y

    a = np.array([x1,y1])
    b = np.array([x2,y2])

    vec = (b-a).T

    normal_vec = np.matmul(np.array([[cos(pi/2),-sin(pi/2)],[sin(pi/2),cos(pi/2)]]), vec)/np.linalg.norm(vec)


    points_line = getEquidistantPoints(a,b,round(np.linalg.norm(vec)/density))

    num_points = round(width/density/2)    # separately in positive and negative direction
    points_in_line = points_line.shape[0]

    all_points = np.zeros((points_in_line*(num_points*2+1),2))

    start_index = points_in_line*num_points     # index of the original point 1 (start)
    goal_index = points_in_line*(num_points+1)-1  # index of the original point 2 (goal)

    all_points[start_index:goal_index+1,:] = points_line

    for i in range(num_points):
        pos_line_points = points_line + normal_vec*density*(i+1)
        neg_line_points = points_line - normal_vec*density*(i+1)
        all_points[points_in_line*(num_points+i+1):points_in_line*(num_points+i+2),:] = pos_line_points
        all_points[points_in_line*(num_points-i-1):points_in_line*(num_points-i),:] = neg_line_points

    all_points = list(map(Point, zip(all_points[:,0], all_points[:,1])))

    return all_points,list(map(Point, zip(points_line[:,0], points_line[:,1]))),start_index,goal_index