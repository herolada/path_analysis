import graph_tool.all as gt
import random
arr = [0,1]

g = gt.Graph()
g.add_vertex(10)

# Normalised RGB color.
# 0->Red, 1->Blue
red_blue_map = {0:(1,0,0,1),1:(0,0,1,1)}
# Create new vertex property
plot_color = g.new_vertex_property('vector<double>')
# add that property to graph
g.vertex_properties['plot_color'] = plot_color
# assign a value to that property for each node of that graph
for v in g.vertices():
    plot_color[v] = red_blue_map[random.choice(arr)]

gt.graph_draw(g,
              vertex_fill_color=g.vertex_properties['plot_color'],output="aaaa.pdf")