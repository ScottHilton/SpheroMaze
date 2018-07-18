# Dijkstra's algorithm for shortest paths
# David Eppstein, UC Irvine, 4 April 2002

# http://aspn.activestate.com/ASPN/Cookbook/Python/Recipe/117228

#modified by Scott Hilton 2018
from priodict import priorityDictionary
from collections import defaultdict

def Dijkstra(G, start, end=None):
    """
    Find shortest paths from the  start vertex to all vertices nearer
    than or equal to the end.

    The input graph G is assumed to have the following representation: A
    vertex can be any object that can be used as an index into a
    dictionary. G is a dictionary, indexed by vertices.  For any vertex
    v, G[v] is itself a dictionary, indexed by the neighbors of v.  For
    any edge v->w, G[v][w] is the length of the edge. This is related to
    the representation in <http://www.python.org/doc/essays/graphs.html>
    where Guido van Rossum suggests representing graphs as dictionaries
    mapping vertices to lists of outgoing edges, however dictionaries of
    edges have many advantages over lists: they can store extra
    information (here, the lengths), they support fast existence tests,
    and they allow easy modification of the graph structure by edge
    insertion and removal. Such modifications are not needed here but
    are important in many other graph algorithms. Since dictionaries
    obey iterator protocol, a graph represented as described here could
    be handed without modification to an algorithm expecting Guido's
    graph representation.

    Of course, G and G[v] need not be actual Python dict objects, they
    can be any other type of object that obeys dict protocol, for
    instance one could use a wrapper in which vertices are URLs of web
    pages and a call to G[v] loads the web page and finds its outgoing
    links.

    The output is a pair (D,P) where D[v] is the distance from start to
    v and P[v] is the predecessor of v along the shortest path from s to
    v.

    Dijkstra's algorithm is only guaranteed to work correctly when all
    edge lengths are positive. This code does not verify this property
    for all edges (only the edges examined until the end vertex is
    reached), but will correctly compute shortest paths even for some
    graphs with negative edges, and will raise an exception if it
    discovers that a negative edge has caused it to make a mistake.
    """

    D = {}  # dictionary of final distances
    P = {}  # dictionary of predecessors
    Q = priorityDictionary()  # estimated distances of non-final vertices
    Q[start] = 0

    for v in Q:
        D[v] = Q[v]
        if v == end:
            break

        for w in G[v]:
            vwLength = D[v] + G[v][w]
            if w in D:
                if vwLength < D[w]:
                    raise ValueError("Dijkstra: found better path to already-final vertex")
            elif w not in Q or vwLength < Q[w]:
                Q[w] = vwLength
                P[w] = v

    return (D, P)


def shortestPath(G, start, end):
    """
    Find a single shortest path from the given start vertex to the given
    end vertex. The input has the same conventions as Dijkstra(). The
    output is a list of the vertices in order along the shortest path.
    """

    D, P = Dijkstra(G, start, end)
    Path = []
    while 1:
        Path.append(end)
        if end == start:
            break
        end = P[end]
    Path.reverse()
    return Path



# def example():
#     positions = set([0,1,2,3,4,5,6,10,11,12,13,14,15,16,20,21,22,23,24,25,26,30,31,32,33,34,35,36])
#     edges = defaultdict(dict)
#     for node in positions:
#         if(node % 10 > 0):
#             edges[node][node - 1] = 1
#         if(node % 10 < 6):
#             edges[node][node + 1] = 1
#         if(node // 10 > 0):
#             edges[node][node - 10] = 1
#         if(node // 10 < 3):
#             edges[node][node + 10] = 1
#     print(edges)
#     print(shortestPath(edges,3,26))
#
# def example2():
#     positions = set([0,1,2,3,4,5,6,10,11,12,13,14,15,16,20,21,22,23,24,25,26,30,31,32,33,34,35,36])
#     edges = defaultdict(set)
#     for node in positions:
#         if(node % 10 > 0):
#             edges[node].add(node - 1)
#         if(node % 10 < 6):
#             edges[node].add(node + 1)
#         if(node // 10 > 0):
#             edges[node].add(node - 10)
#         if(node // 10 < 3):
#             edges[node].add(node + 10)
#
#     print(edges)
#     newEdges = defaultdict(dict)
#     for node in edges:
#         for edge in edges[node]:
#             newEdges[node][edge] = 1
#
#     print(newEdges)
#     print(shortestPath(newEdges,3,26))
#
def example3():
    positions = set([0,1,2,3,4,5,6,10,11,12,13,14,15,16,20,21,22,23,24,25,26,30,31,32,33,34,35,36])
    edges = defaultdict(dict)
    for node in positions:
        if(node % 10 > 0):
            if (True): #there is no wall between node and node - 1
                edges[node][node - 1] = 1
                edges[node - 1][node] = 1
        if(node // 10 > 0):
            if (True): #there is no wall between node and node - 10
                edges[node][node - 10] = 1
                edges[node - 10][node] = 1
    print(edges)
    print(shortestPath(edges,3,26))

if __name__ == '__main__':
    example3()
