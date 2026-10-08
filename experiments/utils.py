import kimera_rpgo


class DataConfig:
    def __init__(self) -> None:
        self.name = None  # Dataset name
        self.pretty_name = None  # Pretty name for plots
        self.no_outlier_g2o = None  # Outlier-Free g2o path
        self.gt_g2o = None  # GT g2o if applicable
        self.outlier_g2o = None  # Test g2o path
        self.outlier_threshold = None
        self.is_3d = False
        self.pcd_dir = None
        self.corrupted_odom = None

    @classmethod
    def from_dict(cls, data: dict):
        c = cls()
        c.name = data["name"]
        c.outlier_g2o = data["outlier_g2o"]
        c.outlier_threshold = data["outlier_threshold"]
        c.is_3d = data["is_3d"]
        if "no_outlier_g2o" in data:
            c.no_outlier_g2o = data["no_outlier_g2o"]
        if "gt_g2o" in data:
            c.gt_g2o = data["gt_g2o"]
        c.pretty_name = data["print"]
        if "pcd_dir" in data:
            c.pcd_dir = data["pcd_dir"]
        if "corrupted_odom" in data:
            c.corrupted_odom = data["corrupted_odom"]
        return c

    def __eq__(self, other):
        return self.name == other.name


class ExpConfig:
    def __init__(self) -> None:
        self.name = None  # Config name
        self.pretty_name = None  # Pretty name for plots
        self.robust_cost = None  # robust cost to use
        self.solver_type = None  # solver type
        self.use_gnc = False  # Use GNC
        self.gnc_barcsq = 0.0  # GNC inlier threshold
        self.gnc_cost = None  # GNC robust cost
        self.use_pcm = False  # Use PCM
        self.max_iterations = 1000
        self.relative_error_tol = 1e-8
        self.absolute_error_tol = 1e-8
        self.error_tol = 0.0

    @classmethod
    def from_dict(cls, data: dict):
        c = cls()
        c.name = data["name"]
        c.robust_cost = data["robust_cost"]
        c.solver_type = data["solver_type"]
        c.pretty_name = data["print"]
        if "use_gnc" in data:
            c.use_gnc = data["use_gnc"]
        if c.use_gnc:
            c.gnc_barcsq = data["gnc_barcsq"] if "gnc_barcsq" in data else 0.0
            c.gnc_cost = data["robust_cost"]
            c.robust_cost = "none"
            # TODO(Yun) a little fragile here: GNC has built in robust cost
        if "use_pcm" in data:
            c.use_pcm = data["use_pcm"]

        # Other optimizer params
        if "max_iterations" in data:
            c.max_iterations = data["max_iterations"]
        if "relative_error_tol" in data:
            c.relative_error_tol = data["relative_error_tol"]
        if "absolute_error_tol" in data:
            c.absolute_error_tol = data["absolute_error_tol"]
        if "error_tol" in data:
            c.error_tol = data["error_tol"]
        return c


# TODO(Yun) make this more general?
def label_factors(
    robust_cost: str,
    gt_pg: kimera_rpgo.utils.Graph,
    res_pg: kimera_rpgo.utils.Graph,
    log_json,
):
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

    if robust_cost == "huber" or robust_cost == "tls" or robust_cost == "none":
        cutoff = 1 - 1e-03
    elif robust_cost == "gm":
        cutoff = 0.25
    else:
        assert False, "Unsupported Robust Cost"
    for i, w in enumerate(weights):
        if i < num_true_inliers:
            if w > cutoff:
                res_pg.edges[i].debug = kimera_rpgo.utils.DebugType.TRUE_NEGATIVE
            else:
                res_pg.edges[i].debug = kimera_rpgo.utils.DebugType.FALSE_POSITIVE
        else:
            if w > cutoff:
                res_pg.edges[i].debug = kimera_rpgo.utils.DebugType.FALSE_NEGATIVE
            else:
                res_pg.edges[i].debug = kimera_rpgo.utils.DebugType.TRUE_POSITIVE
