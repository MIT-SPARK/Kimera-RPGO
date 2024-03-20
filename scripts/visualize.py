import utils
import viz_utils
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse


def main():
    parser = argparse.ArgumentParser(
        description="Visualize G2O")
    parser.add_argument("input", help="input g2o file")
    parser.add_argument('--is_3d', action='store_true')
    parser.add_argument('--is_2d', dest='is_2d', action='store_false')
    parser.set_defaults(is_3d=False)
    args = parser.parse_args()

    color_mapping = {utils.NodeType.POSE: "black", utils.NodeType.LANDMARK: "red",
                     utils.EdgeType.ODOM: "gray", utils.EdgeType.LANDMARK: "orange",
                     utils.EdgeType.LOOP_CLOSURE: "blue"}

    pose_graph = utils.read_pose_graph_from_g2o(args.input, args.is_3d)
    if args.is_3d:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        viz_utils.visualize_graph_3d(ax, pose_graph, color_mapping)
        fig.tight_layout()
        plt.show()
    else:
        fig = plt.figure()
        ax = fig.add_subplot(111)
        viz_utils.visualize_graph_2d(ax, pose_graph, color_mapping)
        fig.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
