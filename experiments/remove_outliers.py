import argparse
import json

import kimera_rpgo.utils as utils


def rearrange_edges(orig_graph, log_json, add_at_end):
    # TODO(Yun) remove this chunck of duplicated code (from label_factors in utils)
    weights = log_json["solver_final_weights"][
        :-1
    ]  # only if fix first pose is true TODO(Yun) account for this
    if log_json["pcm_inliers"]:
        weights = []
        for i in range(len(orig_graph.edges)):
            if i not in log_json["pcm_inliers"]:
                weights.append(0)
                continue
            idx = log_json["pcm_inliers"].index(i)
            weights.append(log_json["solver_final_weights"][idx])
    assert len(weights) == len(orig_graph.edges)  # prior not saved to g2o
    new_graph = orig_graph.copy()
    inliers = []
    outliers = []
    for i, w in enumerate(weights):
        if w > 0.5:  # inlier
            inliers.append(new_graph.edges[i])
        else:  # outlier
            outliers.append(new_graph.edges[i])
    new_graph.edges = inliers
    if add_at_end:
        new_graph.edges.extend(outliers)
    return new_graph


def main():
    parser = argparse.ArgumentParser(description="Generate new G2O with outliers")
    parser.add_argument("input", help="input g2o file")
    parser.add_argument("output", help="output g2o file")
    parser.add_argument("log_json", help="result log containing weights")
    parser.add_argument("--add_at_end", action="store_true")
    parser.add_argument("--no_outliers", dest="add_at_end", action="store_false")
    parser.add_argument("--is_3d", action="store_true")
    parser.add_argument("--is_2d", dest="is_3d", action="store_false")
    parser.set_defaults(is_3d=False, add_at_end=False)
    args = parser.parse_args()

    orig_pg = utils.read_pose_graph_from_g2o(args.input, args.is_3d)
    with open(args.log_json, newline="") as jsonfile:
        log_data = json.load(jsonfile)
    new_pg = rearrange_edges(orig_pg, log_data, args.add_at_end)
    new_pg.write_to_g2o(args.output, args.is_3d)


if __name__ == "__main__":
    main()
