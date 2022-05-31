from graph_tool.all import *
import numpy as np
import time

""" def generate_edges(graph,points,mask):
    cost = 1

    for i in len(range(points)):
        for j in len(range(points)):
            if mask[i:j]:
                 """


""" g = Graph()

x = np.arange(10)
y = np.arange(10)
mask = v_arr == 0

g.add_vertex(v_arr.size)


generate_edges(g,v_arr,mask) """

x = np.arange(10)
y = np.arange(10)

x,y = np.meshgrid(x,y)
x = np.reshape(x,(x.size,1))
y = np.reshape(y,(y.size,1))

points = np.concatenate((x,y),axis=1)
print(points.shape)
g, pos = geometric_graph(points, 1.5)

edges = g.get_edges()
prop = g.new_edge_property("double")

for e in edges:
    print(e[0],e[1])
    prop[g.edge(e[0],e[1])] = 0.1


#graph_draw(g, pos=pos, edge_text=g.edge_index, vertex_text=g.vertex_index, output="geometric.pdf")