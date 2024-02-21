import utils
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse

color_mapping = {utils.NodeType.POSE: "black", utils.NodeType.LANDMARK: "red",
                 utils.EdgeType.ODOM: "gray", utils.EdgeType.LANDMARK: "orange", utils.EdgeType.LOOP_CLOSURE: "blue"}


def visualize_graph_2d(graph):
    nodes_xy = []
    edges_xy = []
    nodes_colors = []
    edges_colors = []
    for node in graph.nodes.values():
        nodes_xy.append([node.pose.x, node.pose.y])
        nodes_colors.append(color_mapping[node.type])

    for edge in graph.edges:
        node_from = graph.nodes[edge.key_from]
        node_to = graph.nodes[edge.key_to]
        edges_xy.append(([node_from.pose.x, node_from.pose.y], [
                        node_to.pose.x, node_to.pose.y]))
        edges_colors.append(color_mapping[edge.type])

    nodes_xy = np.asarray(nodes_xy)
    edges_xy = np.asarray(edges_xy)

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.scatter(nodes_xy[:, 0], nodes_xy[:, 1], c=nodes_colors)
    for i in range(edges_xy.shape[0]):
        ax.plot(edges_xy[i, :, 0], edges_xy[i, :, 1], color=edges_colors[i])
    fig.tight_layout()
    plt.show()


def visualize_graph_3d(graph):
    nodes_t = []
    edges_t = []
    nodes_colors = []
    edges_colors = []
    for node in graph.nodes.values():
        nodes_t.append(node.pose.t)
        nodes_colors.append(color_mapping[node.type])

    for edge in graph.edges:
        node_from = graph.nodes[edge.key_from]
        node_to = graph.nodes[edge.key_to]
        edges_t.append((node_from.pose.t, node_to.pose.t))
        edges_colors.append(color_mapping[edge.type])

    nodes_t = np.asarray(nodes_t)
    edges_t = np.asarray(edges_t)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(nodes_t[:, 0], nodes_t[:, 1], nodes_t[:, 2], c=nodes_colors)
    for i in range(edges_t.shape[0]):
        ax.plot3D(edges_t[i, :, 0], edges_t[i, :, 1],
                  edges_t[i, :, 2], color=edges_colors[i])
    fig.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Visualize G2O")
    parser.add_argument("input", help="input g2o file")
    parser.add_argument('--is_3d', action='store_true')
    parser.add_argument('--is_2d', dest='is_2d', action='store_false')
    parser.set_defaults(is_3d=False)
    args = parser.parse_args()

    pose_graph = utils.read_pose_graph_from_g2o(args.input, args.is_3d)
    if args.is_3d:
        visualize_graph_3d(pose_graph)
    else:
        visualize_graph_2d(pose_graph)


if __name__ == "__main__":
    main()
