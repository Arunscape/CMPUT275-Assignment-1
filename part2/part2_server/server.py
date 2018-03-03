"""
This program acts as a server for a driving route finder for Edmonton.
The client/server interactions are simulated via stdin and stdout
"""

from graph import Graph
from binary_heap import BinaryHeap
from math import sqrt
from breadth_first_search import get_path
from serial import Serial
from time import sleep

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
        Any two consecutive vertices correspond to some edge in graph.
    """
    #Dijkstra’s Algorithm
    reached=dict() #reached dictionary
    events = BinaryHeap() #stores burning events
    events.insert((start,start),0) #start vertex burns at time 0
    while len(events) > 0:
        (u,v),time = events.popmin()
        if v not in reached:
            reached[v]=u
            for w in graph.neighbours(v):
                #new event, edge (v,w) started burning
                events.insert((v,w),time+cost.distance((v,w)))
    return get_path(reached,start,dest)

def load_edmonton_graph(filename):
    """
    Loads the graph of Edmonton from the given file.
    Returns two items:
        graph: the instance of the class Graph() corresponding to the
        directed graph from edmonton-roads-2.0.1.txt
        location: a dictionary mapping the identifier of a vertex to
        the pair (lat, lon) of geographic coordinates for that vertex.
        These should be integers measuring the lat/lon in 100000-ths
        of a degree.
    """

    g = Graph()
    location = dict()

    with open(filename, 'r') as infile:
        for line in infile:
            split_line = line.split(",")

            if split_line[0] == "V":
                g.add_vertex(int(split_line[1]))

                # converting the raw lat/lon to ints in 100000-ths of a degree
                pair = (int(float(split_line[2])*100000),int(float(split_line[3])*100000))
                location[int(split_line[1])] = pair

            # undirected graph
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
    """
    Find and return the closest vertex to a set of coordinates
    Args:
        location: a dictionary mapping vertices to their lat/lon coordinates
        coords: a tuple containing the lat/lon coords you want to find the
        nearest vertex to
    """

    min_distance = float('inf')
    closest = 0 # just to initialize it
    for v in location:
        # Euclidean distance
        distance = sqrt((coords[0]-location[v][0])**2 + (coords[1]-location[v][1])**2)
        if distance <= min_distance:
            closest = v
            min_distance = distance

    return closest

def decode_string(line):
    line_string = line.decode("ASCII")
    stripped = line_string.rstrip("\r\n")
    stripped = stripped.split()
    return stripped

def server_talk():

    with Serial("/dev/ttyACM0", baudrate=9600, timeout=1) as ser:
        while True:
            line = ser.readline()

            if not line:
                print("timeout for initial request, restarting")
                continue

            decoded = decode_string(line)
            print(decoded)

            if decoded[0] != 'R' or len(decoded) != 5: #if an invalid request is received
                print("invalid request, restarting")
                continue

            print("Request received")
            startvertex = find_nearest_vertex(location, (int(decoded[1]),int(decoded[2])) )
            endvertex = find_nearest_vertex(location, (int(decoded[3]),int(decoded[4])) )

            path = least_cost_path(yeg_graph, startvertex, endvertex, cost)
            print(len(path))
            out_line = "N " + str(len(path)) + "\n"
            ser.write(out_line.encode("ASCII"))

            # if theres no path to the destination, return to waiting for a request
            # without acknowledgement
            if len(path) == 0:
                continue


            # wait for acknowledgement
            line = ser.readline()

            if not line:
                print("timeout after num waypoints sent, restarting")
                continue

            decoded = decode_string(line)

            if decoded[0] != 'A' or len(decoded) != 1:
                print("acknowledgement of num waypoints not received, restarting")
                continue

            timeout = False
            for waypoint in path:
                print("Sending waypoints")
                print(waypoint)
                out_line = "W " + str(location[waypoint][0]) + " " + str(location[waypoint][1]) + "\n"
                ser.write(out_line.encode("ASCII"))

                line = ser.readline()

                if not line:
                    print("timeout in acknowledgement of waypoint, restarting")
                    timeout = True
                    break

                decoded = decode_string(line)
                if decoded[0] != 'A' or len(decoded) != 1:
                    print("acknowledgement char not received when waypoint sent, restarting")
                    timeout = True
                    break

            # don't send the E character if no acknowledgement was received
            if timeout:
                continue

            #done sending all the waypoints, return to request waiting state
            print('E')
            out_line = "E" + "\n"
            ser.write(out_line.encode("ASCII"))

if __name__ == "__main__":
    yeg_graph, location = load_edmonton_graph("edmonton-roads-2.0.1.txt")
    cost = CostDistance(location)

    server_talk()
