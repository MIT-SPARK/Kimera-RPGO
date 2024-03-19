import utils
import numpy as np
import argparse
import copy
import random


def generate_outlier_loop_closure(pose_graph, is_3d, covar):
    pose_nodes = []
    for node in pose_graph.nodes:
        if pose_graph.nodes[node].type != utils.NodeType.POSE:
            continue
        pose_nodes.append(node)

    # sample random nodes
    from_node = random.choice(pose_nodes)
    to_node = random.choice(pose_nodes)
    while abs(to_node - from_node) < 2:
        to_node = random.choice(pose_nodes)

    # generate edge
    if not is_3d:
        pose = utils.Pose2D.random()
    else:
        pose = utils.Pose3D.random()

    return utils.Edge(from_node, to_node,
                      utils.EdgeType.LOOP_CLOSURE, pose, covar)


def generate_outlier_landmark_meas(pose_graph, is_3d, covar):
    pose_nodes = []
    landmark_nodes = []
    for node in pose_graph.nodes:
        if pose_graph.nodes[node].type == utils.NodeType.LANDMARK:
            landmark_nodes.append(node)
        if pose_graph.nodes[node].type == utils.NodeType.POSE:
            pose_nodes.append(node)

    # sample random nodes
    from_node = random.choice(pose_nodes)
    to_node = random.choice(landmark_nodes)

    # generate edge
    if not is_3d:
        pose = utils.Pose2D.random(theta_min=0, theta_max=0)
    else:
        pose = utils.Pose3D.random(identity_rot=True)
        pose.R = Rot

    return utils.Edge(from_node, to_node,
                      utils.EdgeType.LANDMARK, pose, covar)


def add_outliers(pose_graph, ratio, is_3d):
    # Count number of loop closures (non-seq pose-pose)
    # Count number of landmark measurements (pose-landmark)
    # And generate outliers for each based on ratio
    loop_closures = 0
    landmark_meas = 0

    lc_covar = None
    ldmk_covar = None

    for edge in pose_graph.edges:
        if edge.type == utils.EdgeType.LOOP_CLOSURE:
            loop_closures += 1
            lc_covar = edge.covar
        if edge.type == utils.EdgeType.LANDMARK:
            landmark_meas += 1
            ldmk_covar = edge.covar

    outlier_loop_closures = int(loop_closures * ratio)
    outlier_landmark_meas = int(landmark_meas * ratio)

    outlier_pose_graph = pose_graph.copy()
    for i in range(outlier_loop_closures):
        outlier_pose_graph.add_edge(
            generate_outlier_loop_closure(pose_graph, is_3d, lc_covar))

    for i in range(outlier_landmark_meas):
        outlier_pose_graph.add_edge(
            generate_outlier_landmark_meas(pose_graph, is_3d, ldmk_covar))
    return outlier_pose_graph


def main():
    parser = argparse.ArgumentParser(
        description="Generate new G2O with outliers")
    parser.add_argument("input", help="input g2o file")
    parser.add_argument("output", help="output g2o file")
    parser.add_argument('--ratio', type=float, default=0.5)
    parser.add_argument('--is_3d', action='store_true')
    parser.add_argument('--is_2d', dest='is_2d', action='store_false')
    parser.set_defaults(is_3d=False)
    args = parser.parse_args()

    pose_graph = utils.read_pose_graph_from_g2o(args.input, args.is_3d)
    corrupted_pose_graph = add_outliers(pose_graph, args.ratio, args.is_3d)

    corrupted_pose_graph.write_to_g2o(args.output, args.is_3d)


if __name__ == "__main__":
    main()
