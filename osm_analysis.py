# %%
from cmath import nan
#from matplotlib.patches import Polygon
import overpy
import OSMPythonTools.api as osm
import shapely.geometry as geometry
from shapely.ops import nearest_points
from shapely.ops import linemerge, unary_union, polygonize
import matplotlib.pyplot as plt
import sys
import itertools
from geodesy import utm
import requests
from xml.etree import ElementTree
import numpy as np
import geopy.distance
from random import random
import time
from copy import deepcopy
from coords_to_waypoints import coords_to_waypoints
from points_to_graph_points import points_to_graph_points
import graph_tool.all as gt


# %%

OSM_URL = "https://www.openstreetmap.org/api/0.6/way/{}/relations"
TERRAIN_TAGS = ['landuse','natural','public_transport','service']
TERRAIN_VALUES = ['park']
TERRAIN_OR_BARRIER_TAGS = ['leisure']
BARRIER_TAGS = ['waterway','barrier','man_made','building','amenity','sport']
NOT_BARRIER_AREA_VALUES = ['parking']
ROAD_TAGS = ['highway','footway','surface']
FOOTWAY_VALUES = ['footway', 'steps', 'cycleway', 'path']
TAGS_KEY_ONLY = ['building']
OBSTACLE_TAGS = ['historic','amenity','natural','tourism','information']
NOT_OBSTACLE_TAGS = ['addr:country','addr:street']

MAX_ROAD_DIST = 10
MAX_FOOTWAY_DIST = 5
MAX_BARRIER_DIST = 10
MAX_OBSTACLE_DIST = 10

class PointInformation():
    def __init__(self,x=0,y=0):
        self.x = x
        self.y = y
        self.altitude = 0
    
class Way():
    def __init__(self,id=-1,is_area=False,nodes=[],tags=None,line=None,in_out=""):
        self.id = id
        self.is_area = is_area
        self.nodes = nodes
        self.tags = tags
        self.line = line
        self.in_out = in_out
    
    def is_road(self):
        if self.tags.get('highway', None) and not self.tags.get('highway', None) in FOOTWAY_VALUES:
            return True

    def is_footway(self):
        if self.tags.get('highway', None) and self.tags.get('highway', None) in FOOTWAY_VALUES:
            return True
        
    def is_terrain(self):
        if any(tag in TERRAIN_TAGS+TERRAIN_OR_BARRIER_TAGS for tag in self.tags) and not any(tag in BARRIER_TAGS for tag in self.tags):
            return True
    
    def is_barrier(self):
        if any(tag in BARRIER_TAGS for tag in self.tags):
            return True
    
    def tag_selection(self, selection=[]):
        selected = []
        tags_list = list(self.tags.items())
        if selection:
            for i in range(len(self.tags.items())):
                if tags_list[i][0] in selection or tags_list[i][1] in selection:
                    if tags_list[i][0] in TAGS_KEY_ONLY:
                        selected.append(tags_list[i][0])
                    else:
                        selected.append(tags_list[i][1])

        return selected


class Obstacle():
    def __init__(self):
        self.point = geometry.Point()
        self.id = 0
        self.tags = {}
    
    def tag_selection(self):
        selected = []
        tags_list = list(self.tags.items())

        for i in range(len(self.tags.items())):
            if (tags_list[i][0] in OBSTACLE_TAGS or tags_list[i][1] in OBSTACLE_TAGS) and not (tags_list[i][0] in NOT_OBSTACLE_TAGS):
                if tags_list[i][0] in TAGS_KEY_ONLY:
                    selected.append(tags_list[i][0])
                else:
                    selected.append(tags_list[i][1])

        return selected


class Visitor(gt.DijkstraVisitor):

    def __init__(self, name, t):
        self.name = name
        self.t = t
        self.last_time = 0

    def discover_vertex(self, u):
        print("-->", self.name[u], "has been discovered!")
        self.t[u] = self.last_time
        self.last_time += 1

    def examine_edge(self, e):
        print("edge (%s, %s) has been examined..." % \
            (self.name[e.source()], self.name[e.target()]))

    def edge_relaxed(self, e):
        print("edge (%s, %s) has been relaxed..." % \
            (self.name[e.source()], self.name[e.target()]))


class PathAnalysis:
    def __init__(self, in_file):
        self.api = overpy.Overpass()
        self.waypoints = np.genfromtxt(in_file, delimiter=',')
        self.waypoints_wgs = np.genfromtxt(in_file, delimiter=',')
        self.max_lat = np.max(self.waypoints[:,0])
        self.min_lat = np.min(self.waypoints[:,0])
        self.max_long = np.max(self.waypoints[:,1])
        self.min_long = np.min(self.waypoints[:,1])
        self.waypoints_to_utm()
        self.max_x = np.max(self.waypoints[:,0])
        self.min_x = np.min(self.waypoints[:,0])
        self.max_y = np.max(self.waypoints[:,1])
        self.min_y = np.min(self.waypoints[:,1])
        self.points = list(map(geometry.Point, zip(self.waypoints[:,0], self.waypoints[:,1])))
        self.points_information = []
        self.obstacles = []
        self.way_node_ids = set() # nodes to be counted as obstacles
        self.terrain_areas = set()
        self.roads = set()
        self.footways = set()
        self.barriers = set()
        self.barrier_areas = set()
        self.terrain_areas_list = set()
        self.roads_list = set()
        self.footways_list = set()
        self.barriers_list = set()
        self.barrier_areas_list = set()
        self.ways = dict()
    
    def waypoints_to_utm(self):
        for i,waypoint in enumerate(self.waypoints):
            utm_coords = utm.fromLatLong(waypoint[0],waypoint[1])
            waypoint = np.array([utm_coords.easting, utm_coords.northing])
            self.waypoints[i] = waypoint


    def get_way_query(self):
        query = """(way({}, {}, {}, {});
                    >;
                    );
                    out;""".format(self.min_lat,self.min_long,self.max_lat,self.max_long)

        return query
    
    def get_rel_query(self):
        query = """(way({}, {}, {}, {});
                    <;
                    );
                    out;""".format(self.min_lat,self.min_long,self.max_lat,self.max_long)

        return query
    
    def get_node_query(self):
        query = """(node({}, {}, {}, {});
                    );
                    out;""".format(self.min_lat,self.min_long,self.max_lat,self.max_long)

        return query

    def parse_ways(self):
        # https://gis.stackexchange.com/questions/259422/how-to-get-a-multipolygon-object-from-overpass-ql
        """ 1. Phase
            Fill self.ways a dictionary of id:way pairs from all the ways from the query."""

        for way in self.osm_ways_data.ways:
            way_to_store = Way()
            coords = []
            is_area = False

            for node in way.nodes:
                utm_coords = utm.fromLatLong(float(node.lat),float(node.lon))
                self.way_node_ids.add(node.id)
                coords.append([utm_coords.easting, utm_coords.northing])
            
            if coords[0] == coords[-1]:
                is_area = True
            
            way_to_store.id = way.id
            way_to_store.is_area = is_area
            way_to_store.nodes = way.nodes
            way_to_store.tags = way.tags

            if is_area:
                way_to_store.line = geometry.Polygon(coords)
            else:
                way_to_store.line = geometry.LineString(coords)
            
            self.ways[way.id] = way_to_store

    def combine_ways(self,ids):
        ways = []
        for id in ids:
            ways.append(self.ways[id])
        i = 0
        while i < len(ways):
            j = 0
            while j < len(ways):
                if i != j:
                    if (ways[i].nodes[0].id == ways[j].nodes[0].id) and (not ways[i].is_area and not ways[j].is_area):
                        ways[i].nodes.reverse()
                    elif (ways[i].nodes[-1].id == ways[j].nodes[-1].id) and (not ways[i].is_area and not ways[j].is_area):
                        ways[j].nodes.reverse()

                    if ways[i].nodes[-1].id == ways[j].nodes[0].id and (not ways[i].is_area and not ways[j].is_area):
                        
                        combined_line = linemerge([ways[i].line, ways[j].line])

                        new_way = Way()
                        new_way.id = int(-10**15*random())
                        while new_way.id in self.ways.keys():
                            new_way.id = int(-10**15*random())
                        new_way.nodes = ways[i].nodes + ways[j].nodes[1:] 
                        new_way.tags = {**ways[i].tags, **ways[j].tags}
                        new_way.line = combined_line
  
                        if new_way.nodes[0].id == new_way.nodes[-1].id:
                            new_way.is_area = True
                            new_way.line = geometry.Polygon(new_way.line.coords)
                        self.ways[new_way.id] = new_way
                        ways[j] = new_way
                        ids[j] = new_way.id
                        ids.pop(i)
                        ways.pop(i)
                        i -= 1
                        j -= 1
                        break
                j += 1
            i += 1
        
        return ids

    def parse_rels(self):
        """2. Phase
            Needs self.ways DICTIONARY (key is id) with a self.is_area parameter."""
        for rel in self.osm_rels_data.relations:
            if len(rel.members) <= 50:   # A lot of members is very likely some relation we are not interested in.
                inner_ids = []
                outer_ids = []
                keys = self.ways.keys()
                for member in rel.members:
                    if member._type_value == "way":
                        if int(member.ref) in keys:
                            if member.role == "outer":
                                outer_ids.append(int(member.ref))
                            else:
                                inner_ids.append(int(member.ref))

                outer_ids = self.combine_ways(outer_ids)

                for id in outer_ids:
                    way = self.ways[id]
                    if way.is_area:  # If area than make holes in the way's polygon.
                        way.line = geometry.Polygon(way.line.exterior.coords, [self.ways[inner_id].line.exterior.coords for inner_id in inner_ids if self.ways[inner_id].is_area])

                    way.in_out = "outer"
                    way.tags.update(rel.tags)
                    self.ways[id] = way

                for id in inner_ids:
                    way = self.ways[id]
                    way.in_out = "inner"
                    way.tags.update(rel.tags)
                    self.ways[id] = way

    def parse_nodes(self):
        obstacles = []

        for node in self.osm_nodes_data.nodes:
            if not node.id in self.way_node_ids:
                if any([tag in OBSTACLE_TAGS for tag in node.tags]) and not any([tag in NOT_OBSTACLE_TAGS for tag in node.tags]):
                    obstacle = Obstacle()
                    obstacle.id = node.id
                    obstacle.tags = node.tags
                    coords = utm.fromLatLong(float(node.lat),float(node.lon))
                    obstacle.point = geometry.Point([coords.easting, coords.northing])
                    obstacles.append(obstacle)

                    obstacle = self.point_to_area(obstacle)
                    obstacle_way = Way(id=obstacle.id, is_area=True, tags=obstacle.tags, line=obstacle.line)
                    self.barrier_areas.add(obstacle_way)
        
        self.obstacles = obstacles

    def point_to_area(self, pointish):
        area = pointish.point.buffer(2)
        pointish.line = area
        return pointish

    def line_to_area(self, way):
        """ The width of the buffer should depend on,
            the type of way (river x fence, highway x path)... """
        area = way.line.buffer(2)
        way.line = area
        return way

    
    def separate_ways(self):
        """ Separate self.ways (DICT) into LISTS:
            self.terrain_areas  (park, field, sand, parking lot...)
            self.roads          (any highway...)
            self.barrier_areas  (building, polygon fence...)
            self.barriers       (line fence, river...)    """
        for way in self.ways.values():
            if way.is_road():
                self.roads.add(way)
            
            elif way.is_footway():
                self.footways.add(way)

            elif way.is_area:
                if way.is_barrier():
                    self.barrier_areas.add(way)

                if way.is_terrain():
                    self.terrain_areas.add(way)
            
            elif way.is_barrier():
                self.barriers.add(way)
                way = self.line_to_area(way)
                way.is_area = True
                self.barrier_areas.add(way)

            else:
                with open("unclassified_tags.txt",'a+') as f:
                        f.write(str(way.tags)+"\n")
            
    def is_crossing_way(self,way,p1,p2):
        line = geometry.LineString([p1,p2])
        return line.intersects(way.line)
    
    def closest_way(self, ways_list, point, n=1):
        """ Input is a list of ways (Way objects) and a point. """
        """ Output is the closest way and its distance from the point. """
        lines = [way.line for way in ways_list]
        distances = np.array([line.distance(point) for line in lines])
        min_distance = np.amin(distances)
        arg_min_distance = np.argmin(distances)
        way = ways_list[arg_min_distance]

        if n == 1:
            return(way, round(min_distance,2), None)
        elif n > 1:
            arg_n_min_dist = np.argpartition(distances,n)[:n]
            n_close_ways = np.array(ways_list)[arg_n_min_dist]
            return(way, round(min_distance,2), n_close_ways)

    def closest_way_and_is_crossing(self, ways_list, current_point, next_point):
        close_way,dist,n_ways = self.closest_way(ways_list,current_point,4)
        is_crossing = False
        for way in n_ways:
            is_crossing = self.is_crossing_way(way,current_point,next_point)
            if is_crossing:
                break
        return close_way,dist,is_crossing
        
        
    def closest_obstacle(self, obstacles, point):
        distances = [obstacle.point.distance(point) for obstacle in obstacles]
        min_distance = np.amin(distances)
        arg_min_distance = np.argmin(distances)
        obstacle = obstacles[arg_min_distance]    
        return(obstacle, round(min_distance,2))

    def is_point_valid(self, point, objects):
        valid = True
        for way in objects['barrier_areas']:
            if way.line.contains(point):
                valid = False
                break
        return valid
    
    def analyze_point(self, point, objects):
        """ Get terrain under point.
            Get nearest road.
            Get nearest barrier.
            Find out if the point is inside a barrier (e.g. inside a building or a fence). """

        point_information = PointInformation(point.x,point.y)

        """ Analyze terrain. """
        """ for way in objects['terrain_areas']:
            if way.line.contains(point):
                point_information.terrain.append(way.tag_selection(TERRAIN_TAGS+TERRAIN_OR_BARRIER_TAGS)) """
        
        """ Find (if any) the nearest road. """
        """ way,dist,is_crossing = self.closest_way_and_is_crossing(objects['roads_list'], point, next_point)
        if dist <= MAX_ROAD_DIST:
            point_information.roads.append([way,dist])
            point_information.is_crossing_road = is_crossing """
        
        """ Find (if any) the nearest footway. """
        """ way,dist,_ = self.closest_way(objects['footways'], point)
        if dist <= MAX_FOOTWAY_DIST:
            point_information.footways.append([way,dist]) """

        """ Find (if any) the nearest barrier. """
        """ way,dist,is_crossing = self.closest_way_and_is_crossing(objects['barriers'], point, next_point)
        if dist <= MAX_BARRIER_DIST:
            point_information.barriers.append([way,dist])
            point_information.is_crossing_barrier = is_crossing """
        
        """ Find if inside of a barrier area.
            If yes throw away the point. """

        hide_me = self.is_point_valid(point,objects)
        
        """ Find (if any) the nearest obstacle. """
        """ obstacle,dist = self.closest_obstacle(objects['obstacles'],point)
        if dist <= MAX_OBSTACLE_DIST:
            point_information.obstacles.append([obstacle,dist]) """
        
        return point_information,hide_me
    
    def reduce_object(self,objects,d,key,min_x,max_x,min_y,max_y):
        for o in objects:

            if key != 'obstacles':
                bounds = o.line.bounds
            else:
                bounds = o.point.bounds

            if bounds[0] < max_x and bounds[2] > min_x and bounds[1] < max_y and bounds[3] > min_y:
                d[key].append(o)

        return d

    def get_reduced_objects(self,min_x,max_x,min_y,max_y):
        reduced_objects = {'terrain_areas':[],
                            'roads':[],
                            'footways':[],
                            'barriers':[],
                            'barrier_areas':[],
                            'obstacles':[]}

        #reduced_objects = self.reduce_object(self.terrain_areas_list,reduced_objects,'terrain_areas',min_x,max_x,min_y,max_y)
        #reduced_objects = self.reduce_object(self.roads_list,reduced_objects,'roads',min_x,max_x,min_y,max_y)
        #reduced_objects = self.reduce_object(self.footways_list,reduced_objects,'footways',min_x,max_x,min_y,max_y)
        #reduced_objects = self.reduce_object(self.barriers_list,reduced_objects,'barriers',min_x,max_x,min_y,max_y)
        #reduced_objects = self.reduce_object(self.barrier_areas_list,reduced_objects,'barrier_areas',min_x,max_x,min_y,max_y)
        #reduced_objects = self.reduce_object(self.obstacles,reduced_objects,'obstacles',min_x,max_x,min_y,max_y)
        
        for area in self.terrain_areas_list:
            bounds = area.line.bounds
            if bounds[0] < max_x and bounds[2] > min_x and bounds[1] < max_y and bounds[3] > min_y:
                reduced_objects['terrain_areas'].append(area)
        
        for road in self.roads_list:
            bounds = road.line.bounds
            if bounds[0] < max_x and bounds[2] > min_x and bounds[1] < max_y and bounds[3] > min_y:
                reduced_objects['roads'].append(road)

        for footway in self.footways_list:
            bounds = footway.line.bounds
            if bounds[0] < max_x and bounds[2] > min_x and bounds[1] < max_y and bounds[3] > min_y:
                reduced_objects['footways'].append(footway)
        
        for barrier in self.barriers_list:
            bounds = barrier.line.bounds
            if bounds[0] < max_x and bounds[2] > min_x and bounds[1] < max_y and bounds[3] > min_y:
                reduced_objects['barriers'].append(barrier)
        
        for area in self.barrier_areas_list:
            bounds = area.line.bounds

            if bounds[0] < max_x and bounds[2] > min_x and bounds[1] < max_y and bounds[3] > min_y:
                reduced_objects['barrier_areas'].append(area)
        
        for obstacle in self.obstacles:
            bounds = obstacle.point.bounds
            if bounds[0] < max_x and bounds[2] > min_x and bounds[1] < max_y and bounds[3] > min_y:
                reduced_objects['obstacles'].append(obstacle)
        return reduced_objects
    
    def analyze_all_points(self):
        #self.graphs = [Graph(directed=False) for point in self.points]
        THRESHOLD = 10

        last_point = self.points[0]

        for i in range(len(self.points)-1):
            t = time.time()
            current_point = last_point
            next_point = self.points[i+1]
            next_point_index = -1

            graph_points, points_line, _, _ = points_to_graph_points(current_point, next_point, density=1, width=THRESHOLD)
            graph_points_arr = np.array([[p.x,p.y,i] for i,p in enumerate(graph_points)])
            print("points to graph points 1: {}".format(round(time.time() - t,2)))
            t = time.time()
            """ Points to be analyzed are ready. Now get all the objects (roads,obstacles...) in the area. """
            reduced_objects = self.get_reduced_objects(np.min(graph_points_arr[:,0]),np.max(graph_points_arr[:,0]),np.min(graph_points_arr[:,1]),np.max(graph_points_arr[:,1]))
            print("get reduced objects: {}".format(round(time.time() - t,2)))
            t = time.time()
            start_and_finish_valid = False
            k = 2
            while not start_and_finish_valid:
                start_valid = self.is_point_valid(current_point,reduced_objects)
                goal_valid = self.is_point_valid(next_point,reduced_objects)
                if start_valid and goal_valid:
                    start_and_finish_valid = True
                elif start_valid:
                    next_point_index -= 1
                    next_point = points_line[next_point_index]
                    if next_point == current_point or next_point == self.points[i+k-1]:
                        next_point = self.points[i+k]
                        k += 1
                else:
                    print("something is WRONG :(")
                #elif goal_valid:
                #else:
            print("check valid goal: {}".format(round(time.time() - t,2)))
            t = time.time()
            graph_points, points_line, start_index, goal_index = points_to_graph_points(current_point, next_point, density=1, width=10)
            graph_points_arr = np.array([[p.x,p.y,i] for i,p in enumerate(graph_points)])
            graph_points_information = [PointInformation()]*len(graph_points)
            mask = [False] * len(graph_points_information)
            print("points to graph points 2: {}".format(round(time.time() - t,2)))
            t = time.time()
            for k,graph_point in enumerate(graph_points):
                graph_points_information[k],mask[k]  = self.analyze_point(graph_point, reduced_objects)
            print("analyze points: {}".format(round(time.time() - t,2)))
            t = time.time()
            graph_points = graph_points_arr[mask]
            graph, v_position = gt.geometric_graph(graph_points[:,:2], 1.5)

            graph_to_array_index_map = {}          # Keys are graph indices and values are graph_point array indices.
            for r in range(len(graph_points)):
                graph_to_array_index_map[r] = int(graph_points[r,2])

            array_to_graph_index_map = {}         
            for q,r in enumerate(graph_points[:,2]):
                array_to_graph_index_map[int(r)] = q
            
            eprop_cost = graph.new_edge_property("double")

            edges = graph.get_edges()
            
            for e in edges:
                cost = self.get_cost(graph_points_information[graph_to_array_index_map[e[0]]], \
                                    graph_points_information[graph_to_array_index_map[e[1]]]) # the indices of the two points of the edge
                eprop_cost[graph.edge(e[0],e[1])] = cost
            
            graph.edge_properties["cost"] = eprop_cost
            #gt.graph_draw(graph, pos=v_position, vertex_text=graph.vertex_index, output="{}.pdf".format(str(i)))
            
            weights = graph.ep["cost"]

            start_index_graph = array_to_graph_index_map[start_index]
            goal_index_graph = array_to_graph_index_map[goal_index]
            print("prepare graph: {}".format(round(time.time() - t,2)))
            t = time.time()
            shortest_path_vertices,_ = gt.shortest_path(graph, \
                                            graph.vertex(start_index_graph), \
                                            graph.vertex(goal_index_graph), \
                                            weights)
            if not shortest_path_vertices:
                print("Did not find")
            print("find shortest path: {}".format(round(time.time() - t,2)))
            print("------------------")
            color = graph.new_vertex_property("vector<double>")
            for v in graph.vertices():
                if v in shortest_path_vertices:
                    color[v] = (1,0,0,1)
                else:
                    color[v] = (0,0,1,1)

            gt.graph_draw(graph, pos=v_position, vertex_fill_color=color, vertex_text=graph.vertex_index, output="{}_solved.pdf".format(str(i)))
            
            last_point = next_point
            """
            name = graph.new_vertex_property("string")
            for v in graph.vertices():
                name[v] = str(graph.vertex_index[v])

            t = graph.new_vertex_property("int")

            dist,pred = dijkstra_search(graph, weights, graph.vertex(start_index_graph), Visitor(t,name))
            pred_arr = pred.a
            
            shortest_path = [goal_index_graph]
            finished = False
            last_node = goal_index_graph
            while not finished:
                last_node = pred_arr[last_node]
                shortest_path.append(last_node)
                if last_node  == start_index_graph:
                    finished = True """


            """ if np.sum(~np.array(mask)):
                graph_points = graph_points_arr
                graph_points = graph_points[mask]
                fig,ax = plt.subplots(figsize=(3, 3), dpi=200)
                ax.scatter(graph_points[:,0], graph_points[:,1])
                for way in reduced_objects['barrier_areas']:
                    if way.is_area:
                        x,y = way.line.exterior.xy
                    else:
                        x,y = way.line.xy
                    ax.plot(x, y, c='red', linewidth=2)
                ax.axis('equal')
                plt.show() """

        """ for i,point in enumerate(self.points):
            if i < len(self.points)-1:
                point_information = self.analyze_point(point, self.points[i+1], i)
            else:
                point_information = self.analyze_point(point, self.points[i], i)
            self.points_information[i] = point_information
            self.points_information[i].x = point.x
            self.points_information[i].y = point.y """

    def get_cost(self, p1, p2):
        return ((p1.x-p2.x)**2 + (p1.y-p2.y)**2)**(1/2)

    def sets_to_lists(self):
        self.roads_list = list(self.roads)
        self.footways_list = list(self.footways)
        self.barrier_areas_list = list(self.barrier_areas)
        self.terrain_areas_list = list(self.terrain_areas)
        self.barriers_list = list(self.barriers)

    def run(self):
        self.parse_ways()
        self.parse_rels()
        if self.osm_nodes_data:
            self.parse_nodes()
        self.separate_ways()
        self.sets_to_lists()
        self.analyze_all_points()

    def write_to_file(self,fn):
        with open(fn,'w+') as f:
            for point in self.points_information:
                f.write(point.__str__())
                f.write("\n")

    def plot(self):

        def plot_ways(ways, ax_to_plot, color, size, do_fill):
            for way in ways:
                if way.is_area:
                    x,y = way.line.exterior.xy
                else:
                    x,y = way.line.xy
                ax_to_plot.plot(x, y, c=color, linewidth=size)
                
                if do_fill:
                    plt.fill(x,y,c='pink')

        fig,ax = plt.subplots(figsize=(24, 18), dpi=200)

        plot_ways(self.roads, ax, 'grey', 3, False)
        plot_ways(self.terrain_areas, ax, 'royalblue', 3, False)
        plot_ways(self.barriers, ax, 'red', 3, False)
        plot_ways(self.barrier_areas, ax, 'magenta', 3, True)

        if self.obstacles:
            obstacle_coords = np.array([[obstacle.point.x,obstacle.point.y] for obstacle in self.obstacles])
            ax.scatter(obstacle_coords[:,0], obstacle_coords[:,1], c='red', s=10, zorder = 20)
        
        ax.scatter(self.waypoints[:,0],self.waypoints[:,1], c='black', s=20, zorder = 10)
        ax.set_xlim(self.min_x-(self.max_x-self.min_x)*0.2, self.max_x+(self.max_x-self.min_x)*0.2)
        ax.set_ylim(self.min_y-(self.max_y-self.min_y)*0.2, self.max_y+(self.max_y-self.min_y)*0.2)
        plt.show()


if __name__ == "__main__":
    waypoints_file = "waypoints.csv"
    coords_to_waypoints(10, waypoints_file)
    path_analysis = PathAnalysis(waypoints_file)

    # %%
    way_query = path_analysis.get_way_query()
    osm_ways_data = path_analysis.api.query(way_query)

    # %%
    rel_query = path_analysis.get_rel_query()
    osm_rels_data = path_analysis.api.query(rel_query)

    # %%
    #node_query = path_analysis.get_node_query()
    #osm_nodes_data = path_analysis.api.query(node_query)

    print("queries complete")

    # %%
    path_analysis.way_query = way_query
    path_analysis.osm_ways_data = osm_ways_data

    path_analysis.rel_query = rel_query
    path_analysis.osm_rels_data = osm_rels_data

    path_analysis.node_query = node_query
    path_analysis.osm_nodes_data = osm_nodes_data

    path_analysis.run()
    path_analysis.write_to_file('path_analysis.txt')
    path_analysis.plot()



# %%
