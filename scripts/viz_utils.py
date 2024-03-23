import utils
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def visualize_graph_2d(ax, graph, color_mapping):
    nodes_xy = []
    edges_xy = []
    nodes_colors = []
    edges_colors = []
    for node in graph.nodes.values():
        # Only visualize desired types
        if node.type not in color_mapping:
            continue
        nodes_xy.append([node.pose.x, node.pose.y])
        nodes_colors.append(color_mapping[node.type])

    for edge in graph.edges:
        # Only visualize desired types
        if edge.type not in color_mapping:
            continue
        node_from = graph.nodes[edge.key_from]
        node_to = graph.nodes[edge.key_to]
        edges_xy.append(([node_from.pose.x, node_from.pose.y], [
                        node_to.pose.x, node_to.pose.y]))
        edges_colors.append(color_mapping[edge.type])

    nodes_xy = np.asarray(nodes_xy)
    edges_xy = np.asarray(edges_xy)

    if nodes_xy.shape[0] > 0:
        ax.scatter(nodes_xy[:, 0], nodes_xy[:, 1], c=nodes_colors)
    for i in range(edges_xy.shape[0]):
        ax.plot(edges_xy[i, :, 0], edges_xy[i, :, 1], color=edges_colors[i])


def visualize_graph_3d(ax, graph, color_mapping, topdown=False):
    nodes_t = []
    edges_t = []
    nodes_colors = []
    edges_colors = []
    for node in graph.nodes.values():
        # Only visualize desired types
        if node.type not in color_mapping:
            continue
        nodes_t.append(node.pose.t)
        nodes_colors.append(color_mapping[node.type])

    for edge in graph.edges:
        # Only visualize desired types
        if edge.type not in color_mapping:
            continue
        node_from = graph.nodes[edge.key_from]
        node_to = graph.nodes[edge.key_to]
        edges_t.append((node_from.pose.t, node_to.pose.t))
        edges_colors.append(color_mapping[edge.type])

    nodes_t = np.asarray(nodes_t)
    edges_t = np.asarray(edges_t)

    if topdown:
        if nodes_t.shape[0] > 0:
            ax.scatter(nodes_t[:, 0], nodes_t[:, 1], c=nodes_colors)
        for i in range(edges_t.shape[0]):
            ax.plot(edges_t[i, :, 0], edges_t[i, :, 1], color=edges_colors[i])
    else:
        if nodes_t.shape[0] > 0:
            ax.scatter(nodes_t[:, 0], nodes_t[:, 1],
                       nodes_t[:, 2], c=nodes_colors)
        for i in range(edges_t.shape[0]):
            ax.plot3D(edges_t[i, :, 0], edges_t[i, :, 1],
                      edges_t[i, :, 2], color=edges_colors[i])


if __name__ == "__main__":
    main()
