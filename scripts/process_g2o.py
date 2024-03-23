import utils
import numpy as np
import argparse
import copy


def reindex(pose_graph, landmark_prefix='l', index_to_remove=[]):
    old_new_key_map = {}

    last_odom_idx = {}
    last_landmark_idx = -1
    for node_key in pose_graph.nodes:
        prefix, index = utils.split_pg_key(node_key)
        if prefix in index_to_remove:
            continue

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
        if node.key not in old_new_key_map:
            continue
        new_node = copy.deepcopy(node)
        new_node.key = old_new_key_map[node.key]
        new_pose_graph.add_node(new_node)

    for edge in pose_graph.edges:
        if edge.key_from not in old_new_key_map or edge.key_to not in old_new_key_map:
            continue
        new_edge = copy.deepcopy(edge)
        new_edge.key_from = old_new_key_map[edge.key_from]
        new_edge.key_to = old_new_key_map[edge.key_to]
        new_pose_graph.add_edge(new_edge)

    return new_pose_graph


def fix_odom(pose_graph, ref_pose_graph):
    for edge in pose_graph.edges:
        # if edge.type != utils.EdgeType.ODOM:
        #     continue
        ref_T_from = ref_pose_graph.nodes[edge.key_from].pose.to_matrix()
        ref_T_to = ref_pose_graph.nodes[edge.key_to].pose.to_matrix()

        from_T_to = np.linalg.inv(ref_T_from) @ ref_T_to
        posetype = type(edge.pose)
        if posetype == utils.Pose2D:
            noise = posetype.random(x_min=-1e-6, x_max=1e-6, y_min=-1e-6,
                                    y_max=1e-6, theta_min=-1e-8, theta_max=1e8).to_matrix()
        else:
            noise_max = np.ones((3,))*1e-6
            noise = posetype.random(t_min=-noise_max, t_max=noise_max, identity_rot=True).to_matrix()
        # from_T_to = from_T_to @ noise
        edge.pose = posetype.from_matrix(from_T_to)


def main():
    parser = argparse.ArgumentParser(
        description="Reindex G2O to RPGO expected format.")
    parser.add_argument("input", help="input g2o file")
    parser.add_argument("output", help="output g2o file")
    parser.add_argument('--is_3d', action='store_true')
    parser.add_argument('--is_2d', dest='is_2d', action='store_false')
    parser.add_argument('-r', '--remove_indices', nargs='+', default=[])
    parser.add_argument('--reference', type=str, default="")
    parser.set_defaults(is_3d=False)
    args = parser.parse_args()

    pose_graph = utils.read_pose_graph_from_g2o(args.input, args.is_3d)
    processed_pose_graph = reindex(
        pose_graph, index_to_remove=args.remove_indices)

    if len(args.reference) > 0:
        ref_pose_graph = utils.read_pose_graph_from_g2o(
            args.reference, args.is_3d)
        ref_pose_graph = reindex(
            ref_pose_graph, index_to_remove=args.remove_indices)
        fix_odom(processed_pose_graph, ref_pose_graph)

    processed_pose_graph.write_to_g2o(args.output, args.is_3d)


if __name__ == "__main__":
    main()
