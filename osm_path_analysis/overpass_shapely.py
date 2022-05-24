from cmath import nan
#from matplotlib.patches import Polygon
import overpy
import OSMPythonTools.api as osm
import shapely.geometry as geometry
from shapely.ops import nearest_points
from shapely.ops import linemerge, unary_union, polygonize
import matplotlib.pyplot as plt
import sys
import requests
from xml.etree import ElementTree
import numpy as np
import geopy.distance
from coords_to_waypoints import coords_to_waypoints
#from skspatial.objects import Line
#from skspatial.objects import Point

OSM_URL = "https://www.openstreetmap.org/api/0.6/way/{}/relations"
AREA_TERRAIN_TAGS = ['landuse','leisure','natural','public_transport','service']
AREA_BARRIER_TAGS = ['barrier','building','amenity']
HIGHWAY_TAGS = ['highway','surface']
BARRIER_TAGS = ['waterway','barrier','man_made']
TAGS_KEY_ONLY = ['building']
OBSTACLE_TAGS = ['historic','amenity','natural']
NOT_OBSTACLE_TAGS = ['addr:country']

class PointInformation():
    terrain = set()
    inside_barrier = []
    highways = []
    barriers = []
    obstacles = []

    def __str__(self):
        s = ""
        if self.terrain:
            s += "{}\n".format(repr(self.terrain))
        else:
            s += "unknown area\n"

        if self.highways:
            for h in self.highways:
                s += "nearest road   : {} at {} m\n".format(h[0:-1], h[-1])
        else:
            s += "no near (10 m) roads\n"

        if self.barriers:
            for b in self.barriers:
                s += "nearest barrier: {} at {} m\n".format(b[0:-1], b[-1])
        else:
            s += "no near (10 m) barriers\n"

        if self.inside_barrier:
            s += "POINT INSIDE BARRIER: {}\n".format(self.inside_barrier)
        else:
            s += "point is outside\n"
        
        if self.obstacles:
            for h in self.obstacles:
                s += "nearest obstacle   : {} at {} m\n".format(h[0:-1], h[-1])
        else:
            s += "no near (10 m) obstacles\n"

        return s
        
class SuperWay:
    def __init__(self):
        self.nodes = []
        self.id = 0
        self.tags = {}

class Highway(SuperWay):
    def __init__(self):
        super().__init__()
        self.line = geometry.LineString

    def get_lower_bound(self):
        return self.line.bounds[0:2]
    
    def get_upper_bound(self):
        return self.line.bounds[2:4]

class Area(SuperWay):
    def __init__(self):
        super().__init__()
        self.relations = []
        self.relations_checked = False
        self.polygon = geometry.Polygon
        self.is_barrier = False

class Barrier(Highway):
    def __init__(self):
        super().__init__()    

class Obstacle(SuperWay):
    def __init__(self):
        self.point = geometry.Point()
        self.id = 0
        self.tags = {}

class PathAnalysis:
    def __init__(self, in_file):
        self.api = overpy.Overpass()
        self.waypoints = np.genfromtxt(in_file, delimiter=',')
        self.points = list(map(geometry.Point, zip(self.waypoints[:,0], self.waypoints[:,1])))

        self.way_query = self.get_way_query()
        self.osm_ways_data = self.api.query(self.way_query)

        self.node_query = self.get_node_query()
        self.osm_nodes_data = self.api.query(self.node_query)

        print("queries complete")

        self.areas, self.highways, self.barriers = self.parse_ways()
        self.obstacles = self.parse_nodes()

        self.terrain = self.get_terrain()
        print(*self.terrain, sep='\n')
        with open("path_analysis.txt",'w+') as f:
            for line in self.terrain:
                f.write(line.__str__())
                f.write("\n")
        
    def get_barrier_areas(self):
        barrier_areas = []
        for area in self.areas:
            if any(tag in area.tags for tag in AREA_BARRIER_TAGS):
                barrier_areas.append(area)
        return barrier_areas

    def get_way_query(self):
        max_lat = np.max(self.waypoints[:,0])
        min_lat = np.min(self.waypoints[:,0])
        max_long = np.max(self.waypoints[:,1])
        min_long = np.min(self.waypoints[:,1])

        query = """(way({}, {}, {}, {});
                    >;
                    );
                    out;""".format(min_lat,min_long,max_lat,max_long)

        #""" Query including relationships. """
        #query = """(
        #            rel({}, {}, {}, {});
        #            >>;
        #            way({}, {}, {}, {});
        #            >;
        #           );
        #            out;""".format(min_lat,min_long,max_lat,max_long,min_lat,min_long,max_lat,max_long)
        return query
    
    def get_node_query(self):
        max_lat = np.max(self.waypoints[:,0])
        min_lat = np.min(self.waypoints[:,0])
        max_long = np.max(self.waypoints[:,1])
        min_long = np.min(self.waypoints[:,1])

        query = """[bbox:{}, {}, {}, {}];
                    rel; > -> .r;
                    way; > -> .w;
                    ((node; - node.r; ); - node.w; );
                    out;""".format(min_lat,min_long,max_lat,max_long)

        return query

    def parse_ways(self):
        # https://gis.stackexchange.com/questions/259422/how-to-get-a-multipolygon-object-from-overpass-ql
        
        areas = []
        highways = []
        barriers = []

        for way in self.osm_ways_data.ways:
            coords = []
            
            try:
                for node in way.nodes:
                    coords.append([node.lat,node.lon])
                
                if coords[0] == coords[-1]:
                    classified = False

                    if any(tag in AREA_TERRAIN_TAGS for tag in way.tags):
                        area = Area()
                        area.id = way.id
                        area.nodes = way.nodes
                        area.tags = way.tags
                        area.polygon = geometry.Polygon(coords)
                        classified = True
                        areas.append(area)
                    
                    elif any(tag in HIGHWAY_TAGS for tag in way.tags):
                        highway = Highway()       
                        highway.id = way.id
                        highway.nodes = way.nodes
                        highway.tags = way.tags
                        highway.line = geometry.LineString(coords)
                        highways.append(highway)

                    if any(tag in AREA_BARRIER_TAGS for tag in way.tags):
                        area = Area()
                        area.id = way.id
                        area.nodes = way.nodes
                        area.tags = way.tags
                        area.polygon = geometry.Polygon(coords)
                        area.is_barrier = True
                        classified = True
                        areas.append(area)

                    if not classified:
                        area = Area()
                        area.id = way.id
                        area.nodes = way.nodes
                        area.tags = way.tags
                        area.polygon = geometry.Polygon(coords)
                        areas.append(area)
                        with open("unclassified_tags.txt",'a+') as uncl_f:
                            uncl_f.write("area "+str(way.tags)+"\n")

                elif any(tag in HIGHWAY_TAGS for tag in way.tags):
                    highway = Highway()       
                    highway.id = way.id
                    highway.nodes = way.nodes
                    highway.tags = way.tags
                    highway.line = geometry.LineString(coords)
                    highways.append(highway)

                elif any(tag in BARRIER_TAGS for tag in way.tags):
                    barrier = Barrier()
                    barrier.id = way.id
                    barrier.nodes = way.nodes
                    barrier.tags = way.tags
                    barrier.line = geometry.LineString(coords)
                    barriers.append(barrier)
                else:
                    with open("unclassified_tags.txt",'a+') as uncl_f:
                        uncl_f.write("way "+str(way.tags)+"\n")
            except:
                continue

        return areas,highways,barriers       

    def parse_nodes(self):
        obstacles = []

        for node in self.osm_nodes_data.nodes:
            if any(np.logical_and([tag in OBSTACLE_TAGS for tag in node.tags],[tag not in NOT_OBSTACLE_TAGS for tag in node.tags])):
                obstacle = Obstacle()
                obstacle.id = node.id
                obstacle.tags = node.tags
                obstacle.point = geometry.Point([node.lat, node.lon])
                obstacles.append(obstacle)
        
        return obstacles
    
    def closest_way(self, ways, point):
        """ Input is a list of ways (Way objects) and a point. """
        lines = [way.line for way in ways]
        distances = np.array([line.distance(point) for line in lines])  # NOT CORRECT. THIS GEOMETRY DOES NOT TAKE INTO CONSIDERATION THAT WE ARE WORKING WITH WGS...
        min_distance = np.amin(distances)
        arg_min_distance = np.argmin(distances)
        closest_hway = ways[arg_min_distance]
        p = geometry.Point(point.bounds[0:2])

        proj = nearest_points(closest_hway.line, p)[0]

        min_distance = geopy.distance.geodesic(p.bounds[0:2],proj.bounds[0:2]).m
        return(closest_hway, round(min_distance,2))
    
    def closest_area(self, areas, point):
        """ Input is a list of ways (Way objects) and a point. """
        lines = [area.polygon for area in areas]
        """ for l in range(len(lines)):
            if lines[l].contains(point):
                return(areas[l], -1) """

        distances = np.array([line.distance(point) for line in lines if not line.contains(point)])  # NOT CORRECT. THIS GEOMETRY DOES NOT TAKE INTO CONSIDERATION THAT WE ARE WORKING WITH WGS...
        min_distance = np.amin(distances)
        arg_min_distance = np.argmin(distances)
        closest_area = areas[arg_min_distance]
        p = geometry.Point(point.bounds[0:2])

        proj = nearest_points(closest_area.polygon, p)[0]

        min_distance = geopy.distance.geodesic(p.bounds[0:2],proj.bounds[0:2]).m
        return(closest_area, round(min_distance,2))
    
    def closest_obstacle(self, obstacles, point):
        distances = [obstacle.point.distance(point) for obstacle in obstacles]
        min_distance = np.amin(distances)
        arg_min_distance = np.argmin(distances)
        closest_obstacle_point = obstacles[arg_min_distance].point
        min_distance = geopy.distance.geodesic(point.bounds[0:2],closest_obstacle_point.bounds[0:2]).m
    
        return(closest_obstacle_point, round(min_distance,2))
    
    def get_terrain(self):
        
        def parse_tags(tags_to_parse, tags_category):
            tags_to_return = []

            for tag in tags_category:
                if tag in tags_to_parse:
                    if tag in TAGS_KEY_ONLY:
                        tags_to_return.append(tag)
                    else:
                        tags_to_return.append(tags_to_parse[tag])

            return tags_to_return

        terrain = [PointInformation() for i in range(len(self.points))]
        
        for i,point in enumerate(self.points):

            # Find inside which areas the point is.
            for area in self.areas:
                polygon = area.polygon
                still_contains = True
                if polygon.contains(point):
                    if not area.relations_checked:
                        url = OSM_URL.format(area.id)
                        r = requests.get(url)
                        try:
                            data = ElementTree.fromstring(r.content.decode("utf-8"))
                            relations = data.findall('relation')
                            r_tags = []
                            for relation in relations:
                                r_tags = relation.findall('tag')
                                r_members = relation.findall('member')
                                r_members_in_out = [member.attrib['role'] for member in r_members]
                                r_members_ids = [int(member.attrib['ref']) for member in r_members]
                                current_area_index = r_members_ids.index(area.id) 
                                inner_area_indices = [ind for ind in range(len(r_members_in_out)) if r_members_in_out[ind] == "inner"]
                                inner_area_ids = [r_members_ids[ind] for ind in inner_area_indices]
                                if r_members_in_out[current_area_index] == "outer":
                                    if "inner" in r_members_in_out:
                                        area.polygon = geometry.Polygon(area.polygon.exterior.coords, [inner_area.polygon.exterior.coords for inner_area in self.areas if inner_area.id in inner_area_ids])                       
                                    
                                    for tag in r_tags:
                                        key = tag.attrib['k']
                                        value = tag.attrib['v']
                                        area.tags[key] = value
                                    
                                    if not polygon.contains(point):
                                        still_contains = False
                        except Exception as e:
                            print(e)
                            pass
                        area.relations_checked = True

                    # Get terrain of the area.
                    if still_contains:
                        terrain[i].terrain = terrain[i].terrain.union(set(parse_tags(area.tags, AREA_TERRAIN_TAGS)))
                        terrain[i].inside_barrier = terrain[i].inside_barrier + parse_tags(area.tags, AREA_BARRIER_TAGS)
                    still_contains = True

            # Find nearest highway (path,road,sidewalk...).
            if self.highways:
                closest_highway,dist = self.closest_way(self.highways, point)
                
                if dist < 10:
                    highway_n_dist = parse_tags(closest_highway.tags, HIGHWAY_TAGS)+[str(dist)]
                    terrain[i].highways = terrain[i].highways + [highway_n_dist]

            # Find nearest area barrier (fenced area, building).
            barrier_areas = self.get_barrier_areas()
            if barrier_areas:
                closest_area_barrier,dist = self.closest_area(barrier_areas, point)

                if dist < 10:
                    area_barrier_n_dist = parse_tags(closest_area_barrier.tags, AREA_BARRIER_TAGS)+[str(dist)]
                    terrain[i].barriers = terrain[i].barriers + [area_barrier_n_dist]

            # Find nearest barrier (waterway).
            if self.barriers:
                closest_barrier,dist = self.closest_way(self.barriers, point)
                
                if dist < 10:
                    barrier_n_dist = parse_tags(closest_barrier.tags, BARRIER_TAGS)+[str(dist)]
                    terrain[i].barriers = terrain[i].barriers + [barrier_n_dist]
        
            if self.obstacles:
                closest_obstacle,dist = self.closest_obstacle(self.obstacles, point)

                if dist < 10:
                    obstacle_n_dist = parse_tags(closest_obstacle.tags, OBSTACLE_TAGS)+[str(dist)]
                    terrain[i].obstacles = terrain[i].obstacles + [obstacle_n_dist]

        return terrain

    def plot(self):
        xys = []

        for area in self.areas:
            x,y = area.polygon.exterior.xy
            xys.append([x,y])

        for way in xys:
            plt.plot(way[1], way[0], c='blue')

        xys = []
        
        for highway in self.highways:
            x,y = highway.line.xy
            xys.append([x,y])

        for way in xys:
            plt.plot(way[1], way[0], c='pink')

        xys = []
        
        for barrier in self.barriers:
            x,y = barrier.line.xy
            xys.append([x,y])

        for way in xys:
            plt.plot(way[1], way[0], c='red')

        obstacle_coords = [obstacle.point for obstacle in self.obstacles]
        plt.scatter(obstacle_coords[:,1], obstacle_coords[:,0], c='red', s=6)
        
        plt.scatter(self.waypoints[:,1],self.waypoints[:,0], c='green', s=5)
        plt.show()


if __name__=='__main__':
    coords_to_waypoints()
    in_file = sys.argv[1]
    path_analysis = PathAnalysis(in_file)
    path_analysis.plot()