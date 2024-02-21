import csv
import numpy as np
import os
import ctypes
from enum import Enum
from scipy.spatial.transform import Rotation as Rot

keyBits = ctypes.sizeof(ctypes.c_uint64) * 8
chrBits = ctypes.sizeof(ctypes.c_ubyte) * 8
indexBits = keyBits - chrBits
chrMask = np.uint64(np.int64(~np.ubyte(0)) << indexBits)
indexMask = ~chrMask


def split_pg_key(pg_key):
    c_ = chr(np.ubyte(np.int64(np.uint64(pg_key) & chrMask) >> indexBits))
    j_ = np.uint64(pg_key) & indexMask
    return c_, j_


def join_pg_key(c_, j_):
    return np.uint64(ord(c_) << indexBits) + np.uint64(j_)


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
            if args[0].startswith("VERTEX"):
                entries = args[0].split()
                self.x = float(entries[2])
                self.y = float(entries[3])
                if len(entries) == 5:
                    self.theta = float(entries[4])
                else:
                    self.theta = 0
            elif args[0].startswith("EDGE"):
                entries = args[0].split()
                self.x = float(entries[3])
                self.y = float(entries[4])
                if len(entries) == 12:
                    self.theta = float(entries[5])
                else:
                    self.theta = 0
            else:
                raise("Unknwon pose type from g2o string")


class Pose3D:
    def __init__(self, *args):
        if len(args) == 2:
            self.t, self.R = args[0], args[1]
        if len(args) == 1:
            # parse g2o line
            if args[0].startswith("VERTEX"):
                entries = args[0].split()
                self.t = np.array([float(e) for e in entries[2:5]])
                if len(entries) == 9:
                    quat = np.array([float(e) for e in entries[5:9]])
                    self.R = Rot.from_quat(quat)
                else:
                    self.R = Rot.from_quat([0, 0, 0, 1])
            elif args[0].startswith("EDGE"):
                entries = args[0].split()
                self.t = np.array([float(e) for e in entries[3:6]])
                if len(entries) == 31:
                    quat = np.array([float(e) for e in entries[6:10]])
                    self.R = Rot.from_quat(quat)
                else:
                    self.R = Rot.from_quat([0, 0, 0, 1])


class Node:
    def __init__(self, key, node_type, pose):
        self.key = key
        self.type = node_type
        self.pose = pose

    def g2o_str_2d(self):
        entries = []
        if self.type == NodeType.POSE:
            entries.append("VERTEX_SE2")
        elif self.type == NodeType.LANDMARK:
            entries.append("VERTEX_XY")
        else:
            raise("Unhandled node type when writing to g2o")
        entries.append(str(self.key))
        entries.append(str(self.pose.x))
        entries.append(str(self.pose.y))
        if self.type == NodeType.POSE:
            entries.append(str(self.pose.theta))
        return " ".join(entries)

    def g2o_str_3d(self):
        entries = []
        if self.type == NodeType.POSE:
            entries.append("VERTEX_SE3:QUAT")
        elif self.type == NodeType.LANDMARK:
            entries.append("VERTEX_TRACKXYZ")
        else:
            raise("Unhandled node type when writing to g2o")
        entries.append(str(self.key))
        entries.extend([str(self.pose.t[i]) for i in range(3)])
        if self.type == NodeType.POSE:
            quat = self.pose.R.as_quat()
            entries.extend([str(quat[i]) for i in range(4)])
        return " ".join(entries)


class Edge:
    def __init__(self, key_from, key_to, edge_type, pose, covar):
        self.key_from = key_from
        self.key_to = key_to
        self.type = edge_type
        self.pose = pose
        self.covar = covar

    def g2o_str_2d(self):
        entries = []
        if self.type == EdgeType.ODOM or self.type == EdgeType.LOOP_CLOSURE:
            entries.append("EDGE_SE2")
        elif self.type == EdgeType.LANDMARK:
            entries.append("EDGE_SE2_XY")
        else:
            raise("Unhandled edge type when writing to g2o")
        entries.append(str(self.key_from))
        entries.append(str(self.key_to))
        entries.append(str(self.pose.x))
        entries.append(str(self.pose.y))
        if self.type != NodeType.LANDMARK:
            entries.append(str(self.pose.theta))
        entries.extend([str(c) for c in self.covar])
        return " ".join(entries)

    def g2o_str_3d(self):
        entries = []
        if self.type == EdgeType.ODOM or self.type == EdgeType.LOOP_CLOSURE:
            entries.append("EDGE_SE3:QUAT")
        elif self.type == EdgeType.LANDMARK:
            entries.append("EDGE_SE3_TRACKXYZ")
        else:
            raise("Unhandled edge type when writing to g2o")
        entries.append(str(self.key_from))
        entries.append(str(self.key_to))
        entries.extend([str(self.pose.t[i]) for i in range(3)])
        if self.type != EdgeType.LANDMARK:
            quat = self.pose.R.as_quat()
            entries.extend([str(quat[i]) for i in range(4)])
        entries.extend([str(c) for c in self.covar])
        return " ".join(entries)


class Graph:
    def __init__(self):
        self.nodes = {}
        self.edges = []

    def add_node(self, node):
        self.nodes[node.key] = node

    def add_edge(self, edge):
        self.edges.append(edge)

    def write_to_g2o(self, g2o_file, is_3d):
        output = open(g2o_file, "w")
        for node in self.nodes.values():
            if not is_3d:
                node_str = node.g2o_str_2d()
            else:
                node_str = node.g2o_str_3d()
            output.write(node_str + "\n")

        for edge in self.edges:
            if not is_3d:
                edge_str = edge.g2o_str_2d()
            else:
                edge_str = edge.g2o_str_3d()
            output.write(edge_str + "\n")

        output.close()

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
            covar = [float(c) for c in g2o_line.split()[6:12]]
        elif g2o_line.split()[0] == "EDGE_SE2_XY":
            edge_type = EdgeType.LANDMARK
            covar = [float(c) for c in g2o_line.split()[5:8]]
        else:
            raise Exception(
                "Unhandled input edge type {}".format(g2o_line.split()[0]))
        pose = Pose2D(g2o_line)
        return Edge(kf, kt, edge_type, pose, covar)
    else:
        kf = int(g2o_line.split()[1])
        kt = int(g2o_line.split()[2])
        if g2o_line.split()[0] == "EDGE_SE3:QUAT":
            if kt == kf + 1:
                edge_type = EdgeType.ODOM
            else:
                edge_type = EdgeType.LOOP_CLOSURE
            covar = [float(c) for c in g2o_line.split()[10:31]]
        elif g2o_line.split()[0] == "EDGE_SE3_TRACKXYZ":
            edge_type = EdgeType.LANDMARK
            covar = [float(c) for c in g2o_line.split()[6:12]]
        else:
            raise Exception(
                "Unhandled input edge type {}".format(g2o_line.split()[0]))
        pose = Pose3D(g2o_line)
        return Edge(kf, kt, edge_type, pose, covar)


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
