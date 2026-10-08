import argparse

import kimera_rpgo.utils as utils
import matplotlib.pyplot as plt
import numpy as np
import scan_utils
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 -- registers the 3D projection


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
        if edge.type not in color_mapping and edge.debug not in color_mapping:
            continue
        node_from = graph.nodes[edge.key_from]
        node_to = graph.nodes[edge.key_to]
        edges_xy.append(
            ([node_from.pose.x, node_from.pose.y], [node_to.pose.x, node_to.pose.y])
        )
        if edge.debug in color_mapping:
            edges_colors.append(color_mapping[edge.debug])
        else:
            edges_colors.append(color_mapping[edge.type])

    nodes_xy = np.asarray(nodes_xy)
    edges_xy = np.asarray(edges_xy)

    for i in range(edges_xy.shape[0]):
        ax.plot(
            edges_xy[i, :, 0],
            edges_xy[i, :, 1],
            color=edges_colors[i],
            alpha=0.2,
            zorder=1,
        )
    if nodes_xy.shape[0] > 0:
        ax.scatter(nodes_xy[:, 0], nodes_xy[:, 1], c=nodes_colors, s=0.5, zorder=2)


def visualize_graph_3d(ax, graph, color_mapping, topdown=False, pcd_dir=None):
    if pcd_dir is not None:
        keyed_scans = scan_utils.load_scans(pcd_dir)
        keyed_scans.clean_scans()
    nodes_t = []
    edges_t = []
    nodes_colors = []
    edges_colors = []
    scans = []
    for node in graph.nodes.values():
        # Only visualize desired types
        if node.type not in color_mapping:
            continue
        nodes_t.append(node.pose.t)
        if pcd_dir is not None:
            keyed_scans.transform_scan(node.key, node.pose.to_matrix())
            scans.append(keyed_scans.get_scan(node.key))

        nodes_colors.append(color_mapping[node.type])

    for edge in graph.edges:
        # Only visualize desired types
        if edge.type not in color_mapping and edge.debug not in color_mapping:
            continue
        node_from = graph.nodes[edge.key_from]
        node_to = graph.nodes[edge.key_to]
        edges_t.append((node_from.pose.t, node_to.pose.t))
        if edge.debug in color_mapping:
            edges_colors.append(color_mapping[edge.debug])
        else:
            edges_colors.append(color_mapping[edge.type])

    nodes_t = np.asarray(nodes_t)
    edges_t = np.asarray(edges_t)

    if topdown:
        if len(scans) > 0:
            sample = 10  # TODO(Yun) make a parameters
            scan_map = np.concatenate(scans)
            ax.scatter(
                scan_map[::sample, 0],
                scan_map[::sample, 1],
                c=scan_map[::sample, 2],
                s=0.5,
                alpha=0.1,
                zorder=1,
            )

        for i in range(edges_t.shape[0]):
            ax.plot(
                edges_t[i, :, 0],
                edges_t[i, :, 1],
                color=edges_colors[i],
                alpha=0.2,
                zorder=2,
            )

        if nodes_t.shape[0] > 0:
            ax.scatter(nodes_t[:, 0], nodes_t[:, 1], c=nodes_colors, s=0.5, zorder=3)

    else:
        for i in range(edges_t.shape[0]):
            ax.plot3D(
                edges_t[i, :, 0],
                edges_t[i, :, 1],
                edges_t[i, :, 2],
                color=edges_colors[i],
                alpha=0.2,
                zorder=1,
            )
        if nodes_t.shape[0] > 0:
            ax.scatter(
                nodes_t[:, 0],
                nodes_t[:, 1],
                nodes_t[:, 2],
                c=nodes_colors,
                s=0.5,
                zorder=2,
            )


def main():
    parser = argparse.ArgumentParser(description="Visualize G2O")
    parser.add_argument("input", help="input g2o file")
    parser.add_argument("--is_3d", action="store_true")
    parser.add_argument("--is_2d", dest="is_3d", action="store_false")
    parser.set_defaults(is_3d=False)
    args = parser.parse_args()

    color_mapping = {
        utils.NodeType.POSE: "black",
        utils.NodeType.LANDMARK: "red",
        utils.EdgeType.ODOM: "gray",
        utils.EdgeType.LANDMARK: "orange",
        utils.EdgeType.LOOP_CLOSURE: "blue",
    }

    pose_graph = utils.read_pose_graph_from_g2o(args.input, args.is_3d)
    if args.is_3d:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        visualize_graph_3d(ax, pose_graph, color_mapping)
        fig.tight_layout()
        plt.show()
    else:
        fig = plt.figure()
        ax = fig.add_subplot(111)
        visualize_graph_2d(ax, pose_graph, color_mapping)
        fig.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
