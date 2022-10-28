import networkx as nx
import re
import random
from networkx.algorithms.approximation.steinertree import steiner_tree

nodes = "[(1,0), (4,0), (1,3), (-2,0), (1,-3)]"
edges = "[(0,1), (0,2), (0,3), (0,4)]"

nodes = [[float(y) for y in x.replace(")", "").split(",")] for x in re.sub("[\[\]( ]","",nodes).split("),")]
edges = [[int(y) for y in x.replace(")", "").split(",")] for x in re.sub("[\[\]( ]","",edges).split("),")]

G = nx.Graph()
for n, [x, y] in enumerate(nodes):
  G.add_node(n, x=x, y=y)

for n1, n2 in edges:
  G.add_edge(n1, n2)

num_terminals = random.randint(3, G.number_of_nodes())

terminals = random.sample(range(G.number_of_nodes()), num_terminals)

print(terminals)

tree = steiner_tree(G, terminals)

for n in tree.nodes:
  x = tree.nodes[n]['x']
  y = tree.nodes[n]['y']

print(tree)