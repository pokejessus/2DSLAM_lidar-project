import numpy as np
import heapq

class Node:
    def __init__(self, x: float | int, y: float | int):
        self.x = x
        self.y = y
        self.dist = float("inf")
    
    def __lt__(self, other):
        return self.dist < other.dist

class Graph:
    def __init__(self):
        self.ls_nodes = []
        self.ls_edges = []
    
    def add_node(self, node: Node):
        self.ls_nodes.append(node)

    def add_edges(self, index_1: int, index_2: int):

        if (index_1 == index_2):
            raise Exception("Simple graph, can't have loops")
        elif (len(self.ls_nodes) < abs(index_1) or len(self.ls_nodes) < abs(index_2)):
            raise Exception("Index out of range")

        for edge in self.ls_edges:
            if ((self.ls_nodes[index_1], self.ls_nodes[index_2]) == edge):
                raise Exception("Simple graph, can't have two edges that connect the same nodes")

        self.ls_edges.append((self.ls_nodes[index_1], self.ls_nodes[index_2]))


    def delete_edges(self, index_1: int, index_2: int):
        idx = 0
        exists = False
        for i, edge in enumerate(self.ls_edges):
            if ((self.ls_nodes[index_1], self.ls_nodes[index_2]) == edge):
                exists = True
                idx = i
                break

        if not exists:
            raise Exception("Edge does not exist")
        
        self.ls_edges.pop(idx)


def Euclidean_distance(node_1: Node, node_2: Node):
    return np.sqrt((node_1.x - node_2.x)**2 + (node_1.y - node_2.y)**2)

def Dijkstra(graph: Graph, node: Node):
    for v in graph.ls_nodes:
        v.dist = float("inf")
    
    node.dist = 0.
    temp_ls_nodes = [v for v in graph.ls_nodes]

    heapq.heapify(temp_ls_nodes)

    while temp_ls_nodes:
        v = temp_ls_nodes.pop(0)

        


