from graph import Graph
from binary_heap import BinaryHeap
from math import sqrt
from breadth_first_search import get_path

def least_cost_path(graph, start, dest, cost):
    """Find and return a least cost path in graph from start
    vertex to dest vertex.
    Efficiency: If E is the number of edges, the run-time is
    O( E log(E) ).
    Args:
    graph (Graph): The digraph defining the edges between the
    vertices.
    start: The vertex where the path starts. It is assumed
    that start is a vertex of graph.
    dest:  The vertex where the path ends. It is assumed
    that dest is a vertex of graph.
    cost:  A class with a method called "distance" that takes
    as input an edge (a pair of vertices) and returns the cost
    of the edge. For more details, see the CostDistance class
    description below.
    Returns:
    list: A potentially empty list (if no path can be found) of
    the vertices in the graph. If there was a path, the first
    vertex is always start, the last is always dest in the list.
    Any two consecutive vertices correspond to some
    edge in graph.
    """
    reached=dict()
    events = BinaryHeap()
    events.insert((start,start),0)
    while len(events) > 0:
        (u,v),time = events.popmin()
        #print('u:{} , v:{} , time:{} '.format(u,v,time))
        if v not in reached:
            reached[v]=u
            for w in graph.neighbours(v):
                events.insert((v,w),time+cost.distance((v,w)))
    #print(reached)
    return get_path(reached,start,dest)

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

    #hey

    with open(filename, 'r') as infile:
        for line in infile:
            split_line = line.split(",")

            if split_line[0] == "V":
                g.add_vertex(int(split_line[1]))
                pair = (int(float(split_line[2])*100000),int(float(split_line[3])*100000))
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


        u = self.location[e[0]]
        v = self.location[e[1]]

        distance = sqrt((u[0]-v[0])**2 + (u[1]-v[1])**2)

        return distance

def find_nearest_vertex(location, coords):
    ''' Returns the closest vertex to a set of coordinates '''

    min_distance = 0
    for v in location:
        distance = sqrt((coords[0]-location[v][0])**2 + (coords[1]-location[v][1])**2)
        if distance <= min_distance:
            closest = v
            min_distance = distance

    return closest


if __name__ == "__main__":
    yeg_graph, location = load_edmonton_graph("edmonton-roads-2.0.1.txt")
    line = input().split()

    if line[0]=='R':
        startvertex= find_nearest_vertex(location, (line[1],line[2]) )
        endvertex = find_nearest_vertex(location, (line[3],line[4]) )
