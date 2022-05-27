from graph_tool.all import *

g = Graph(directed=False)

vertices = g.add_vertex(10)

graph_draw(g,vertex_text=g.vertex_index)