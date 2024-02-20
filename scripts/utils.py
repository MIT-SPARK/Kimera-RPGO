import csv
import numpy as np
import os
from enum import Enum


def split_pg_key(pg_key):
    c_ = chr(numpy.ubyte(numpy.int64(numpy.uint64(pg_key) & chrMask) >> indexBits))
    j_ = numpy.uint64(pg_key) & indexMask
    return c_, j_


def join_pg_key(c_, j_):
    return numpy.uint64(ord(c_) << indexBits) + numpy.uint64(j_)


class NodeType(Enum):
    ODOM = 0
    LANDMARK = 1
    VERTEX = 2


class EdgeType(Enum):
    ODOM = 0
    LOOP_CLOSURE = 1
    ASSOCIATION = 2


class Node:
    def __init__(self, key, node_type):
        self.key = key
        self.type = node_type


class Edge:
    def __init__(self, key_from, key_to, edge_type):
        self.key_from = key_from
        self.key_to = key_to
        self.type = edge_type


class Graph:
    def __init__(self):
        self.nodes = {}
        self.edges = []

    def add_node(self, node):
        pass

    def add_edge(self, edge):
        pass


def node_from_g2o_vertex(g2o_line):
    if line.startswith("VERTEX_SE2"):
        pass
        return node
    if line.startswith("VERTEX_SE3"):
        pass
        return node


def edge_from_g2o_edge(g2o_line):
    if line.startswith("EDGE_SE2"):
        pass
        return edge

    if line.startswith("EDGE_SE3"):
        pass
        return edge


def read_traj_from_g2o(filename):
    if not os.path.exists(filename):
        print("Cannot find {}. ".format(filename))
        return []

    f = open(filename, "r")
    pose_graph = Graph()

    for line in f:
        if line.startswith("VERTEX"):
            node = node_from_g2o_vertex(line)
            pose_graph.add_node(node)
        if line.startswith("EDGE"):
            edge = edge_from_g2o_edge(g2o_line)
            pose_graph.add_edge(edge)

    return pose_graph
