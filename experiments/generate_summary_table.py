import argparse
import json
import os

import kimera_rpgo
import metric_utils
import yaml
from tabulate import tabulate

from utils import DataConfig, ExpConfig


class TableGenerator:
    def __init__(self, output_dir):
        self.datasets = []
        self.output_dir = output_dir
        self.configurations = []
        self.result_dirs = {}  # {config: path}

    @classmethod
    def from_yaml_list(cls, yaml_list: list[str], output_dir: str):
        c = cls(output_dir)
        for yaml_file in yaml_list:
            with open(yaml_file, "r") as stream:
                yaml_data = yaml.safe_load(stream)

                datasets = yaml_data["datasets"]
                for data in datasets:
                    data_conf = DataConfig.from_dict(data)
                    if data_conf not in c.datasets:
                        c.datasets.append(data_conf)

                configs = yaml_data["configs"]
                for conf in configs:
                    exp_config = ExpConfig.from_dict(conf)
                    c.configurations.append(exp_config)
                    c.result_dirs[exp_config] = yaml_data["output_dir"]
        return c

    def generate_table(self) -> None:
        # Entries
        table_data = {}  # {data: {config: {metrics: }}}
        for j, dataset in enumerate(self.datasets):
            data_results = {}  # {config: {metrics: }}
            for i, conf in enumerate(self.configurations):
                config_results = {}
                label_dir = os.path.join(self.result_dirs[conf], dataset.name)
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

                # Get RMSE to reference
                config_results["rmse"] = metric_utils.compute_rmse(gt_pg, res_pg)

                # Get timing
                config_results["elapsed"] = log_data["elapsed_ms"]

                # Get outlier recall and precision
                config_results["precision"], config_results["recall"] = (
                    metric_utils.compute_precision_recall(gt_pg, res_pg, log_data)
                )

                data_results[conf] = config_results
            table_data[dataset.name] = data_results

            # For now save a table for each dataset (to merge
            table = []
            for conf, res in data_results.items():
                table.append(
                    [
                        conf.pretty_name,
                        res["rmse"],
                        res["precision"],
                        res["recall"],
                        res["elapsed"],
                    ]
                )

            t = tabulate(
                table,
                headers=[
                    "Config",
                    "RMSE (m)",
                    "Precision (%)",
                    "Recall (%)",
                    "Time (ms)",
                ],
            )

            table_path = os.path.join(self.output_dir, f"table_{dataset.name}.txt")
            with open(table_path, "w") as f:
                f.write(t)


def main():
    parser = argparse.ArgumentParser(description="Comparison Experiment")
    parser.add_argument("config_dir", help="Directory containing config yamls")
    parser.add_argument("output_dir", help="Ouput directory")
    args = parser.parse_args()

    config_files = [
        os.path.join(args.config_dir, f)
        for f in os.listdir(args.config_dir)
        if f.endswith(".yaml")
    ]
    generator = TableGenerator.from_yaml_list(config_files, args.output_dir)
    generator.generate_table()


if __name__ == "__main__":
    main()
