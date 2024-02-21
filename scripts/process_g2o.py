import utils
import numpy as np
import argparse
import copy


def reindex(pose_graph, landmark_prefix='l'):
    old_new_key_map = {}

    last_odom_idx = {}
    last_landmark_idx = -1
    for node_key in pose_graph.nodes:
        prefix, index = utils.split_pg_key(node_key)
        node = pose_graph.nodes[node_key]
        if node.type == utils.NodeType.POSE:
            if prefix not in last_odom_idx:
                last_odom_idx[prefix] = -1
            old_new_key_map[node_key] = utils.join_pg_key(
                prefix, last_odom_idx[prefix] + 1)
            last_odom_idx[prefix] += 1

        if node.type == utils.NodeType.LANDMARK:
            old_new_key_map[node_key] = utils.join_pg_key(
                landmark_prefix, last_landmark_idx + 1)
            last_landmark_idx += 1

    new_pose_graph = utils.Graph()
    for node in pose_graph.nodes.values():
        new_node = copy.deepcopy(node)
        new_node.key = old_new_key_map[node.key]
        new_pose_graph.add_node(new_node)

    for edge in pose_graph.edges:
        new_edge = copy.deepcopy(edge)
        new_edge.key_from = old_new_key_map[edge.key_from]
        new_edge.key_to = old_new_key_map[edge.key_to]
        new_pose_graph.add_edge(new_edge)

    return new_pose_graph


def main():
    parser = argparse.ArgumentParser(
        description="Visualize G2O")
    parser.add_argument("input", help="input g2o file")
    parser.add_argument("output", help="output g2o file")
    parser.add_argument('--is_3d', action='store_true')
    parser.add_argument('--is_2d', dest='is_2d', action='store_false')
    parser.set_defaults(is_3d=False)
    args = parser.parse_args()

    pose_graph = utils.read_pose_graph_from_g2o(args.input, args.is_3d)
    processed_pose_graph = reindex(pose_graph)

    processed_pose_graph.write_to_g2o(args.output, args.is_3d)

if __name__ == "__main__":
    main()
