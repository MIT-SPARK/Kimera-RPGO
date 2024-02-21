import csv
import numpy as np
import os
from enum import Enum
from scipy.spatial.transform import Rotation as Rot


def split_pg_key(pg_key):
    c_ = chr(numpy.ubyte(numpy.int64(numpy.uint64(pg_key) & chrMask) >> indexBits))
    j_ = numpy.uint64(pg_key) & indexMask
    return c_, j_


def join_pg_key(c_, j_):
    return numpy.uint64(ord(c_) << indexBits) + numpy.uint64(j_)


class NodeType(Enum):
    POSE = 0
    LANDMARK = 1


class EdgeType(Enum):
    ODOM = 0
    LOOP_CLOSURE = 1
    LANDMARK = 2
    ASSOCIATION = 3


class Pose2D:
    def __init__(self, *args):
        if len(args) == 3:
            self.x, self.y, self.theta = args[0], args[1], args[2]
        if len(args) == 1:
            # parse g2o line
            entries = args[0].split()
            self.x = float(entries[2])
            self.y = float(entries[3])
            if len(entries) == 5:
                self.theta = float(entries[4])
            else:
                self.theta = 0


class Pose3D:
    def __init__(self, *args):
        if len(args) == 2:
            self.t, self.R = args[0], args[1]
        if len(args) == 1:
            # parse g2o line
            # VERTEX_SE3:QUAT 6989586621679009798 6.7564 -0.503973 1.89705 -0.0427225 -0.0831353 0.0310157 0.995139
            entries = args[0].split()
            self.t = np.array([float(e) for e in entries[2:5]])
            if len(entries) == 9:
                quat = np.array([float(e) for e in entries[5:9]])
                self.R = Rot.from_quat(quat)
            else:
                self.R = Rot.from_quat([0, 0, 0, 1])


class Node:
    def __init__(self, key, node_type, pose):
        self.key = key
        self.type = node_type
        self.pose = pose


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
        self.nodes[node.key] = node

    def add_edge(self, edge):
        self.edges.append(edge)


def node_from_g2o_vertex(g2o_line, is_3d):
    if not is_3d:
        if g2o_line.startswith("VERTEX_SE2"):
            node_type = NodeType.POSE
        elif g2o_line.startswith("VERTEX_XY"):
            node_type = NodeType.LANDMARK
        else:
            raise Exception(
                "Unhandled input node type {}".format(g2o_line.split()[0]))
        key = int(g2o_line.split()[1])
        pose = Pose2D(g2o_line)
        return Node(key, node_type, pose)
    else:
        if g2o_line.startswith("VERTEX_SE3:QUAT"):
            node_type = NodeType.POSE
        elif g2o_line.startswith("VERTEX_TRACKXYZ"):
            node_type = NodeType.LANDMARK
        else:
            raise Exception(
                "Unhandled input node type {}".format(g2o_line.split()[0]))
        key = int(g2o_line.split()[1])
        pose = Pose3D(g2o_line)
        return Node(key, node_type, pose)


def edge_from_g2o_edge(g2o_line, is_3d):
    if not is_3d:
        kf = int(g2o_line.split()[1])
        kt = int(g2o_line.split()[2])
        if g2o_line.split()[0] == "EDGE_SE2":
            if kt == kf + 1:
                edge_type = EdgeType.ODOM
            else:
                edge_type = EdgeType.LOOP_CLOSURE
        elif g2o_line.split()[0] == "EDGE_SE2_XY":
            edge_type = EdgeType.LANDMARK
        else:
            raise Exception(
                "Unhandled input edge type {}".format(g2o_line.split()[0]))
        return Edge(kf, kt, edge_type)
    else:
        kf = int(g2o_line.split()[1])
        kt = int(g2o_line.split()[2])
        if g2o_line.split()[0] == "EDGE_SE3:QUAT":
            if kt == kf + 1:
                edge_type = EdgeType.ODOM
            else:
                edge_type = EdgeType.LOOP_CLOSURE
        elif g2o_line.split()[0] == "EDGE_SE3_TRACKXYZ":
            edge_type = EdgeType.LANDMARK
        else:
            raise Exception(
                "Unhandled input edge type {}".format(g2o_line.split()[0]))
        return Edge(kf, kt, edge_type)


def read_pose_graph_from_g2o(filename, is_3d):
    if not os.path.exists(filename):
        print("Cannot find {}. ".format(filename))
        return []

    f = open(filename, "r")
    pose_graph = Graph()

    for line in f:
        if line.startswith("VERTEX"):
            node = node_from_g2o_vertex(line, is_3d)
            pose_graph.add_node(node)
        if line.startswith("EDGE"):
            edge = edge_from_g2o_edge(line, is_3d)
            pose_graph.add_edge(edge)

    return pose_graph
