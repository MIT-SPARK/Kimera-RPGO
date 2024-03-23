import utils
import viz_utils
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse

est_color_mapping = {utils.NodeType.LANDMARK: "red",
                     utils.EdgeType.ODOM: "blue"}
ref_color_mapping = {utils.NodeType.LANDMARK: "grey",
                     utils.EdgeType.ODOM: "grey"}


def align_pose_graph_origin(pose_graph, ref_pose_graph):
    first_key = min(ref_pose_graph.nodes.keys())
    T_w_est = pose_graph.nodes[first_key].pose.to_matrix()
    T_w_ref = ref_pose_graph.nodes[first_key].pose.to_matrix()
    T_est_ref = T_w_ref @ np.linalg.inv(T_w_est)
    T_est_ref[:2, 0:2] = np.eye(2)
    for key in pose_graph.nodes:
        pose_graph.nodes[key].pose = pose_graph.nodes[key].pose.transform(
            T_est_ref)


def main():
    parser = argparse.ArgumentParser(
        description="Plot and compare results.")
    parser.add_argument("input", help="input result folder")
    parser.add_argument("reference", help="reference g2o")
    parser.add_argument('-e', '--eval_list', nargs='+', default=[])
    parser.add_argument('-l', '--label', nargs='+', default=[])
    parser.add_argument('--is_3d', action='store_true')
    parser.add_argument('--is_2d', dest='is_2d', action='store_false')
    parser.set_defaults(is_3d=False)
    args = parser.parse_args()

    fig = plt.figure()
    ref_pose_graph = utils.read_pose_graph_from_g2o(args.reference, args.is_3d)
    for i in range(len(args.eval_list)):
        input_g2o = "{}/{}/result.g2o".format(args.input, args.eval_list[i])
        pose_graph = utils.read_pose_graph_from_g2o(input_g2o, args.is_3d)
        align_pose_graph_origin(pose_graph, ref_pose_graph)
        if i >= len(args.label):
            label = args.eval_list[i]
        else:
            label = args.label[i]

        if args.is_3d:
            # ax = fig.add_subplot(1, len(args.eval_list), i+1, projection="3d")
            ax = fig.add_subplot(1, len(args.eval_list), i+1)
            viz_utils.visualize_graph_3d(ax, ref_pose_graph, ref_color_mapping, topdown=True)
            viz_utils.visualize_graph_3d(ax, pose_graph, est_color_mapping, topdown=True)
            ax.set_title(label)
        else:
            ax = fig.add_subplot(1, len(args.eval_list), i+1)
            viz_utils.visualize_graph_2d(ax, ref_pose_graph, ref_color_mapping)
            viz_utils.visualize_graph_2d(ax, pose_graph, est_color_mapping)
            ax.set_title(label)
    plt.show()


if __name__ == "__main__":
    main()
