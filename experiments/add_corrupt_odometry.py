import argparse

import kimera_rpgo.utils as utils
import numpy as np


def add_corrupt_odom(pose_graph: utils.Graph, corrupt_odom_idx: int) -> utils.Graph:
    corrupted_pose_graph = pose_graph.copy()
    shift_nodes_after = 0
    corrupt_meas_pose = None
    delta_2d = utils.Pose2D.random(
        x_min=5.0, x_max=10.0, y_min=5.0, y_max=10.0, theta_min=0.5, theta_max=1.0
    )
    delta_3d = utils.Pose3D.random(
        t_min=np.array([10, 10, 10]), t_max=np.array([20, 20, 20])
    )
    for i, edge in enumerate(corrupted_pose_graph.edges):
        if i == corrupt_odom_idx:
            if type(edge.pose) is utils.Pose2D:
                edge.pose = utils.Pose2D.from_matrix(
                    edge.pose.to_matrix() @ delta_2d.to_matrix()
                )
            else:
                edge.pose = utils.Pose3D.from_matrix(
                    edge.pose.to_matrix() @ delta_3d.to_matrix()
                )
            shift_nodes_after = edge.key_from
            break
    for key in corrupted_pose_graph.nodes:
        if key <= shift_nodes_after:
            continue

        node = corrupted_pose_graph.nodes[key]
        if key == shift_nodes_after + 1:
            corrupt_meas_pose = node.pose.to_matrix()

        if node.type != utils.NodeType.POSE:
            continue

        corrupt_T_node = np.linalg.inv(corrupt_meas_pose) @ node.pose.to_matrix()
        if type(node.pose) is utils.Pose2D:
            node.pose = utils.Pose2D.from_matrix(
                corrupt_meas_pose @ delta_2d.to_matrix() @ corrupt_T_node
            )
        else:
            node.pose = utils.Pose3D.from_matrix(
                corrupt_meas_pose @ delta_3d.to_matrix() @ corrupt_T_node
            )

    return corrupted_pose_graph


def main():
    parser = argparse.ArgumentParser(description="Generate perturbed pose graph")
    parser.add_argument("input", help="input g2o file")
    parser.add_argument("output", help="output g2o file")
    parser.add_argument("corrupt_odom_idx", type=int, help="index of odom to corrupt")
    parser.add_argument("--is_3d", action="store_true")
    parser.add_argument("--is_2d", dest="is_3d", action="store_false")
    parser.set_defaults(is_3d=False)
    args = parser.parse_args()

    pose_graph = utils.read_pose_graph_from_g2o(args.input, args.is_3d)
    corrupted_pose_graph = add_corrupt_odom(pose_graph, args.corrupt_odom_idx)

    corrupted_pose_graph.write_to_g2o(args.output, args.is_3d)


if __name__ == "__main__":
    main()
