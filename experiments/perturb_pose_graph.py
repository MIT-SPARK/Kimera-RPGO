import argparse

import kimera_rpgo.utils as utils
import numpy as np


def perturb_graph(
    pose_graph: utils.Graph, drift_t: float, drift_R: float, is_3d: bool
) -> utils.Graph:
    perturbed_pose_graph = pose_graph.copy()
    for key in perturbed_pose_graph.nodes:
        prefix, index = utils.split_pg_key(key)
        node = perturbed_pose_graph.nodes[key]
        orig_pose = node.pose
        posetype = type(node.pose)
        if posetype == utils.Pose2D:
            t_delta = drift_t * index
            R_delta = drift_R * index
            noise = posetype.random(
                x_min=-t_delta,
                x_max=t_delta,
                y_min=-t_delta,
                y_max=t_delta,
                theta_min=-R_delta,
                theta_max=R_delta,
            ).to_matrix()
        else:
            noise_max = np.ones((3,)) * index * drift_t
            # TODO: perturb rot 3D
            noise = posetype.random(
                t_min=-noise_max, t_max=noise_max, identity_rot=True
            ).to_matrix()
        new_pose = orig_pose.transform(noise)
        perturbed_pose_graph.nodes[key].pose = new_pose

    return perturbed_pose_graph


def main():
    parser = argparse.ArgumentParser(description="Generate perturbed pose graph")
    parser.add_argument("input", help="input g2o file")
    parser.add_argument("output", help="output g2o file")
    parser.add_argument("--drift_t", type=float, default=1.0e-04)
    parser.add_argument("--drift_R", type=float, default=1.0e-05)
    parser.add_argument("--is_3d", action="store_true")
    parser.add_argument("--is_2d", dest="is_3d", action="store_false")
    parser.set_defaults(is_3d=False)
    args = parser.parse_args()

    pose_graph = utils.read_pose_graph_from_g2o(args.input, args.is_3d)
    perturbed_pose_graph = perturb_graph(
        pose_graph, args.drift_t, args.drift_R, args.is_3d
    )

    perturbed_pose_graph.write_to_g2o(args.output, args.is_3d)


if __name__ == "__main__":
    main()
