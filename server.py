from graph import Graph


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
