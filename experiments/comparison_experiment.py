import argparse
import json
import os
import shutil

import kimera_rpgo
import matplotlib.pyplot as plt
import viz_g2o
import yaml

from utils import DataConfig, ExpConfig, label_factors

plt.rcParams.update({"font.size": 12})

color_mapping = {
    kimera_rpgo.utils.NodeType.POSE: "black",
    kimera_rpgo.utils.NodeType.LANDMARK: "black",
    kimera_rpgo.utils.EdgeType.ODOM: "black",
    kimera_rpgo.utils.EdgeType.LANDMARK: "gray",
    kimera_rpgo.utils.EdgeType.LOOP_CLOSURE: "gray",
    kimera_rpgo.utils.DebugType.FALSE_POSITIVE: "blue",
    kimera_rpgo.utils.DebugType.FALSE_NEGATIVE: "red",
}


class ComparisonExperimentConfig:
    def __init__(self):
        self.datasets = []
        self.output_dir = None
        self.generate_plot = True
        self.configurations = []
        self.rpgo_configs = {}
        self.save_separate = False

    @classmethod
    def from_yaml(cls, yaml_file: str):
        c = cls()
        with open(yaml_file, "r") as stream:
            yaml_data = yaml.safe_load(stream)
            c.generate_plot = yaml_data["generate_plot"]
            c.save_separate = yaml_data["save_separate"]
            c.output_dir = yaml_data["output_dir"]

            datasets = yaml_data["datasets"]
            for data in datasets:
                c.datasets.append(DataConfig.from_dict(data))

            configs = yaml_data["configs"]
            for conf in configs:
                exp_config = ExpConfig.from_dict(conf)
                c.configurations.append(exp_config)
                cost = kimera_rpgo.config_utils.cost_from_str(exp_config.robust_cost)
                config = kimera_rpgo.RpgoConfig()
                if exp_config.solver_type == "least_squares":
                    config.solver_type = kimera_rpgo.SolverType.LEAST_SQUARES
                    config.solver_config.loadDefaultLeastSquaresConfig()
                elif exp_config.solver_type == "gradient":
                    config.solver_type = kimera_rpgo.SolverType.GRADIENT
                    config.solver_config.loadDefaultGradientConfig()

                if exp_config.use_gnc:
                    config.use_gnc = True
                    config.solver_config.gnc_params.barc_sq = exp_config.gnc_barcsq
                    config.solver_config.gnc_params.robust_cost = (
                        kimera_rpgo.config_utils.cost_from_str(exp_config.gnc_cost)
                    )

                if exp_config.use_pcm:
                    config.use_pcm = True
                    config.pcm_config.metric_type = kimera_rpgo.PcmMetricType.COVARIANCE
                    # set mahalanobis distance to outlier_threshold below
                config.loss_type = cost
                config.solver_config.optimizer_params.maxIterations = (
                    exp_config.max_iterations
                )
                config.solver_config.optimizer_params.relativeErrorTol = (
                    exp_config.relative_error_tol
                )
                config.solver_config.optimizer_params.absoluteErrorTol = (
                    exp_config.absolute_error_tol
                )
                config.solver_config.optimizer_params.errorTol = exp_config.error_tol
                c.rpgo_configs[exp_config.name] = config
        return c


class ComparisonExperiment:
    def __init__(self, config: ComparisonExperimentConfig) -> None:
        self.config = config

    def initialize_output_dir(self, dir_path: str) -> None:
        if os.path.isdir(dir_path):
            return
        else:
            os.makedirs(dir_path)

    def generate_reference(self) -> None:
        print("Generate references...")
        # Run outlier free g2o with least squares to generate reference results
        for dataset in self.config.datasets:
            print(f"Generating reference result... Dataset: {dataset.pretty_name}")
            label_dir = os.path.join(self.config.output_dir, dataset.name)
            self.initialize_output_dir(label_dir)
            gt_g2o = os.path.join(label_dir, "result_gt.g2o")
            if os.path.exists(gt_g2o):
                continue
            # If ground truth provided, directly use as gt
            if dataset.gt_g2o:
                shutil.copyfile(dataset.gt_g2o, gt_g2o)
                continue
            config = kimera_rpgo.RpgoConfig()
            config.solver_type = kimera_rpgo.SolverType.LEAST_SQUARES
            config.solver_config.loadDefaultLeastSquaresConfig()
            runner = kimera_rpgo.Rpgo(config)
            runner.loadG2o(dataset.no_outlier_g2o, dataset.is_3d)
            runner.fixFirstPose(dataset.is_3d)
            runner.run()
            runner.writeResult(gt_g2o)

    def run(self) -> None:
        print("Run experiments...")
        # Run experiments
        for dataset in self.config.datasets:
            for conf in self.config.configurations:
                print(
                    f"Running experiment... Dataset: {dataset.pretty_name} Config: {conf.pretty_name}"
                )
                # Setup RPGO and run
                label_dir = os.path.join(self.config.output_dir, dataset.name)
                self.initialize_output_dir(label_dir)
                g2o_name = f"result_{conf.name}.g2o"
                log_name = f"log_{conf.name}.json"
                result_g2o = os.path.join(label_dir, g2o_name)
                log_file = os.path.join(label_dir, log_name)
                if os.path.exists(result_g2o):
                    continue

                config = self.config.rpgo_configs[conf.name]
                # TODO(Yun) this is subtle, make more clear
                if dataset.outlier_threshold is not None:
                    config.loss_threshold_c = dataset.outlier_threshold
                    config.solver_config.gnc_params.barc_sq = (
                        dataset.outlier_threshold**2
                    )
                    config.pcm_config.mahalanobis_threshold = dataset.outlier_threshold
                runner = kimera_rpgo.Rpgo(config)
                runner.loadG2o(dataset.outlier_g2o, dataset.is_3d)
                runner.fixFirstPose(dataset.is_3d)
                if dataset.corrupted_odom is not None:
                    for idx in dataset.corrupted_odom:
                        runner.setCorruptedOdom(idx)
                runner.run()
                runner.writeResult(result_g2o)
                runner.writeLog(log_file)

    def plot(self) -> None:
        if not self.config.generate_plot:
            return

        num_datasets = len(self.config.datasets)
        num_configs = len(self.config.configurations)

        fig = plt.figure(dpi=1200)

        # Plot reference / gt
        for j, dataset in enumerate(self.config.datasets):
            label_dir = os.path.join(self.config.output_dir, dataset.name)
            gt_g2o = os.path.join(label_dir, "result_gt.g2o")
            gt_pg = kimera_rpgo.utils.read_pose_graph_from_g2o(gt_g2o, dataset.is_3d)
            if dataset.is_3d:
                if self.config.save_separate:
                    ax = fig.add_subplot(1, 1, 1)
                else:
                    ax = fig.add_subplot(num_configs + 1, num_datasets, j + 1)
                viz_g2o.visualize_graph_3d(
                    ax, gt_pg, color_mapping, topdown=True, pcd_dir=dataset.pcd_dir
                )
            else:
                if self.config.save_separate:
                    ax = fig.add_subplot(1, 1, 1)
                else:
                    ax = fig.add_subplot(num_configs + 1, num_datasets, j + 1)
                viz_g2o.visualize_graph_2d(ax, gt_pg, color_mapping)
            # ax.set_xticks([])
            # ax.set_yticks([])
            ax.axis("equal")
            if self.config.save_separate:
                fig.tight_layout()
                plt.savefig(
                    os.path.join(self.config.output_dir, f"{dataset.name}_gt.pdf"),
                    bbox_inches="tight",
                )
                fig = plt.figure(dpi=1200)
            else:
                ax.set_xlabel(dataset.pretty_name)
                ax.xaxis.set_label_position("top")
                if j == 0:
                    ax.set_ylabel("Ground Truth")

        # Plot results
        for j, dataset in enumerate(self.config.datasets):
            for i, conf in enumerate(self.config.configurations):
                label_dir = os.path.join(self.config.output_dir, dataset.name)
                gt_g2o = os.path.join(label_dir, "result_gt.g2o")
                gt_pg = kimera_rpgo.utils.read_pose_graph_from_g2o(
                    gt_g2o, dataset.is_3d
                )
                res_g2o = os.path.join(label_dir, f"result_{conf.name}.g2o")
                res_pg = kimera_rpgo.utils.read_pose_graph_from_g2o(
                    res_g2o, dataset.is_3d
                )
                log_json = os.path.join(label_dir, f"log_{conf.name}.json")
                with open(log_json, newline="") as jsonfile:
                    log_data = json.load(jsonfile)
                label_factors(conf.robust_cost, gt_pg, res_pg, log_data)
                if dataset.is_3d:
                    if self.config.save_separate:
                        ax = fig.add_subplot(1, 1, 1)
                    else:
                        ax = fig.add_subplot(
                            num_configs + 1,
                            num_datasets,
                            (i + 1) * num_datasets + j + 1,
                        )

                    viz_g2o.visualize_graph_3d(
                        ax, res_pg, color_mapping, topdown=True, pcd_dir=dataset.pcd_dir
                    )
                else:
                    if self.config.save_separate:
                        ax = fig.add_subplot(1, 1, 1)
                    else:
                        ax = fig.add_subplot(
                            num_configs + 1,
                            num_datasets,
                            (i + 1) * num_datasets + j + 1,
                        )
                    viz_g2o.visualize_graph_2d(ax, res_pg, color_mapping)

                # ax.set_xticks([])
                # ax.set_yticks([])
                ax.axis("equal")
                if self.config.save_separate:
                    fig.tight_layout()
                    plt.savefig(
                        os.path.join(
                            self.config.output_dir,
                            f"{dataset.name}_{conf.name}.pdf",
                        ),
                        bbox_inches="tight",
                    )
                    fig = plt.figure(dpi=1200)
                else:
                    if j == 0:
                        ax.set_ylabel(conf.pretty_name)
        if not self.config.save_separate:
            fig.tight_layout()
            plt.show()

    def plot_log(self) -> None:
        fig = plt.figure()

        # Plot results
        for j, dataset in enumerate(self.config.datasets):
            for i, conf in enumerate(self.config.configurations):
                label_dir = os.path.join(self.config.output_dir, dataset.name)
                gt_g2o = os.path.join(label_dir, "result_gt.g2o")
                gt_pg = kimera_rpgo.utils.read_pose_graph_from_g2o(
                    gt_g2o, dataset.is_3d
                )
                res_g2o = os.path.join(label_dir, f"result_{conf.name}.g2o")
                res_pg = kimera_rpgo.utils.read_pose_graph_from_g2o(
                    res_g2o, dataset.is_3d
                )
                log_json = os.path.join(label_dir, f"log_{conf.name}.json")
                with open(log_json, newline="") as jsonfile:
                    log_data = json.load(jsonfile)
                label_factors(conf.robust_cost, gt_pg, res_pg, log_data)
        fig.tight_layout()
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Comparison Experiment")
    parser.add_argument("yaml", help="Experiment config yaml")
    parser.add_argument(
        "--plot_convergence", action="store_true", help="Plot convergence"
    )
    args = parser.parse_args()

    config = ComparisonExperimentConfig.from_yaml(args.yaml)
    experiment = ComparisonExperiment(config)
    experiment.generate_reference()
    experiment.run()
    experiment.plot()
    if args.plot_convergence:
        experiment.plot_log()


if __name__ == "__main__":
    main()
