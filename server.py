from graph import Graph
from binary_heap import BinaryHeap

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
        if v not in reached:
            reached[v]=u
            for w in graph.neighbours(v):
                events.insert((v,w),time+cost.distance((v,w)))
    #print(reached)
    return reached

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
    pass

class CostDistance:
    """
    A class with a method called distance that will return the Euclidean
    between two given vertices.
    """
    def __init__(self, location):
        """
        Creates an instance of the CostDistance class and stores the
        3
        dictionary "location" as a member of this class.
        """
        pass
    def distance(self, e):
        """
        Here e is a pair (u,v) of vertices.
        Returns the Euclidean distance between the two vertices u and v.
        """
        pass


if __name__ == "__main__":
    graph = Graph({1,2,3,4,5,6}, [(1,2), (1,3), (1,6), (2,1),(2,3), (2,4), (3,1), (3,2), (3,4), (3,6), (4,2), (4,3), (4,5), (5,4), (5,6), (6,1), (6,3), (6,5)])
    #
    #lengths of the edges described explicitly
    weights = {(1,2): 7, (1,3):9, (1,6):14, (2,1):7, (2,3):10,
               (2,4):15, (3,1):9, (3,2):10, (3,4):11, (3,6):2,
               (4,2):15, (4,3):11, (4,5):6, (5,4):6, (5,6):9, (6,1):14,
               (6,3):2, (6,5):9}

    cost=CostDistance(.123)

    least_cost_path(graph,1,3,cost)
    # print()
