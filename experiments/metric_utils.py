import kimera_rpgo
import numpy as np


def compute_rmse(gt_pg: kimera_rpgo.utils.Graph, res_pg: kimera_rpgo.utils.Graph):
    squared_total_error = 0
    num_poses = 0
    for node_id in gt_pg.nodes:
        gt_node = gt_pg.nodes[node_id]
        res_node = res_pg.nodes[node_id]

        delta = gt_node.pose.trans() - res_node.pose.trans()
        squared_total_error += delta.T @ delta
        num_poses += 1

    return np.sqrt(squared_total_error / num_poses)


def compute_precision_recall(
    gt_pg: kimera_rpgo.utils.Graph, res_pg: kimera_rpgo.utils.Graph, log_json
):
    # Compute precision and recall of identifying OUTLIERS
    num_true_inliers = len(gt_pg.edges)
    weights = log_json["solver_final_weights"][:-1]
    if log_json["pcm_inliers"]:
        weights = []
        for i in range(len(res_pg.edges)):
            if i not in log_json["pcm_inliers"]:
                weights.append(0)
                continue
            idx = log_json["pcm_inliers"].index(i)
            weights.append(log_json["solver_final_weights"][idx])
    assert len(weights) == len(res_pg.edges)  # prior not saved to g2o

    # This is in terms of the outliers (true positive: correctly identify as outlier in contrast to correctly id as true inlier)
    true_positives = 0
    true_negatives = 0
    false_positives = 0
    false_negatives = 0
    for i, w in enumerate(weights):
        if i < num_true_inliers:
            if w > 0.5:
                true_negatives += 1
            else:
                false_positives += 1
        else:
            if w > 0.5:
                false_negatives += 1
            else:
                true_positives += 1

    if true_positives == 0:
        return 0, 0
    return 100 * true_positives / (
        true_positives + false_positives
    ), 100 * true_positives / (true_positives + false_negatives)
