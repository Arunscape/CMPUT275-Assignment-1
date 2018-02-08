from graph import Graph
from binary_heap import BinaryHeap
from math import sqrt


def load_edmonton_graph(filename):
    """
    Loads the graph of Edmonton from the given file.
    Returns two items
    graph: the instance of the class Graph() corresponding to the
    directed graph from edmonton-roads-2.0.1.txt
    location: a dictionary mapping the identifier of a vertex to
    the pair (lat, lon) of geographic coordinates for that vertex.
    These should be integers measuring the lat/lon in 100000-ths
    of a degree.
    In particular, the return statement in your code should be
    return graph, location
    (or whatever name you use for the variables).
    Note: the vertex identifiers should be converted to integers
    before being added to the graph and the dictionary.
    """

    g = Graph()
    location = dict()

    with open(filename, 'r') as infile:
        for line in infile:
            split_line = line.split(",")

            if split_line[0] == "V":
                g.add_vertex(int(split_line[1]))
                pair = (int(float(split_line[2])*10000),int(float(split_line[3])*10000))
                location[int(split_line[1])] = pair

            if split_line[0] == "E":
                g.add_edge((int(split_line[1]),int(split_line[2])))

    return g, location

class CostDistance:
    """
    A class with a method called distance that will return the Euclidean
    between two given vertices.
    """
    def __init__(self, location):
        """
        Creates an instance of the CostDistance class and stores the
        dictionary "location" as a member of this class.
        """

        self.location = location


    def distance(self, e):
        """
        Here e is a pair (u,v) of vertices.
        Returns the Euclidean distance between the two vertices u and v.
        """

        x = location[u]
        y = location[v]

        distance = sqrt((x[0]-x[1])**2 + (y[0]-y[1])**2)

        return distance
