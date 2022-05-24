from ast import arg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 
import numpy as np
from coords_to_waypoints import coords_to_waypoints
import pyproj
from geodesy import utm
import shapely.geometry as geometry
from shapely.ops import nearest_points
from os import path
import numpy.ma as ma

class sjtsk2utm():
    def __init__(self,file_name):
        self.file_name = file_name
        self.data_sjtsk = np.loadtxt(file_name, delimiter=" ")
        self.sjtsk = pyproj.Proj("epsg:5514")
        #self.wgs = pyproj.Proj("epsg:4326")
        self.utm = pyproj.Proj("+proj=utm zone=33 north")

        self.waypoints = np.loadtxt("waypoints.csv",delimiter=",")
        self.waypoints = [utm.fromLatLong(self.waypoints[i,0],self.waypoints[i,1]) for i in range(len(self.waypoints))]
        self.waypoints = np.array([[self.waypoints[i].easting, self.waypoints[i].northing] for i in range(len(self.waypoints))])

    def run(self):
        self.data_utm = (np.array(list(pyproj.transform(self.sjtsk, self.utm, self.data_sjtsk[:,0], self.data_sjtsk[:,1])) + [self.data_sjtsk[:,2]])).T

    def save_to_file(self):
        np.savetxt(self.file_name[:-3]+"csv", self.data_utm, delimiter=",")

    def plot(self):
        plt.scatter(self.data_utm[::100,0], self.data_utm[::100,1], alpha = 0.5)
        plt.scatter(self.waypoints[:,0], self.waypoints[:,1], c='red', alpha = 0.5)
        plt.show()

class DataPoints():
    def __init__(self, fn_waypoints, fn_data_points, n_closest):
        """ Prepare waypoints to analyze. """
        self.waypoints = np.genfromtxt(fn_waypoints, delimiter=',')
        self.waypoints_to_utm()
        self.waypoints = np.pad(self.waypoints, ((0,0), (0,1)), constant_values=(0,0))  # add a column of zeros

        self.max_easting = np.max(self.waypoints[:,0])
        self.min_easting = np.min(self.waypoints[:,0])
        self.max_northing = np.max(self.waypoints[:,1])
        self.min_northing = np.min(self.waypoints[:,1])

        """ Prepare data. """
        self.data_points = np.genfromtxt(fn_data_points, delimiter=',')
        
        """ Parameter for height assignment. """
        self.n_closest = n_closest

    def waypoints_to_utm(self):
        for i,waypoint in enumerate(self.waypoints):
            utm_coords = utm.fromLatLong(waypoint[0],waypoint[1])
            waypoint = np.array([utm_coords.easting, utm_coords.northing])
            self.waypoints[i] = waypoint

    def narrow_region(self):
        mask =  (self.data_points[:,0] > self.min_easting-5)  & \
                (self.data_points[:,0] < self.max_easting+5)  & \
                (self.data_points[:,1] > self.min_northing-5) & \
                (self.data_points[:,1] < self.max_northing+5)

        self.data_points = self.data_points[mask]
    
    def closest_points(self, point, points, n):
        """ Find the n closest points to point and their distances. """
        points = np.asarray(points)
        dist2 = np.sum((points - point)**2, axis=1)
        arg_n_min_dist = np.argpartition(dist2,n)[:n]
        #n_closest_points = np.array(points)[arg_n_min_dist]
        distances = np.sqrt(dist2[arg_n_min_dist])
        return [arg_n_min_dist,distances]

    def assign_height(self):
        """ To each waypoint assign height as the weighted mean of heights of the n
        closest points, where the weights are 1/distance of point from waypoint."""
        for i,point in enumerate(self.waypoints):
            arg_n_min_dist,distances = self.closest_points(point[0:2], self.data_points[:,0:2], self.n_closest)
            n_closest_points = self.data_points[arg_n_min_dist]
            heights = n_closest_points[:,2]
            weights = np.reciprocal(distances)
            height = np.sum(np.multiply(heights,weights/np.sum(weights)))
            self.waypoints[i,2] = height

    def visualize(self):
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.scatter(self.waypoints[:,0], self.waypoints[:,1], self.waypoints[:,2])
        ax.set_xlabel('Easting [m]')
        ax.set_ylabel('Northing [m]')
        ax.set_zlabel('Altitude [m]')
        plt.show()
    
    def plot_points(self):
        plt.scatter(self.data_points[:,0], self.data_points[:,1], c='blue', alpha = 0.5)
        plt.scatter(self.waypoints[:,0], self.waypoints[:,1], c='red', alpha = 0.5)
        plt.show()

    def run(self):
        self.narrow_region()
        self.assign_closest_point_height()
        self.plot_points()
        self.visualize()

if __name__ == "__main__":
    fn_4g = "553988_CVUT/4g/PRAH72_4g.xyz"
    fn_5g = "553988_CVUT/5g/PRAH72_5g.xyz"

    fn = fn_5g

    waypoints_file = "waypoints.csv"
    coords_to_waypoints(0.1, waypoints_file)

    """ Transform points from SJTSK format to UTM. Save as csv. """
    if not path.exists(fn[:-3]+"csv"):
        transformer = sjtsk2utm(fn)
        transformer.run()
        transformer.save_to_file()
    
    """ Prepare class for handling the data points and waypoints.
        Narrow the region to the smallest rectangle containing all the waypoints.
        This is necessary becuase the region is unnecessarily large and thus slow for computations. """
    data = DataPoints("waypoints.csv",fn[:-3]+"csv",4)
    data.run()

    """ For each waypoint find the n closest data points and calculate the weighted mean
        of their heights, where the weight for each point is 1/distance from waypoint. """
