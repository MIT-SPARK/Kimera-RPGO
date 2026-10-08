import os

import numpy as np
import open3d as o3d


class KeyedScans:
    def __init__(self) -> None:
        self.keyed_scans = {}

    def add_scan(self, key: int, scan: o3d.geometry.PointCloud):
        self.keyed_scans[key] = scan

    def get_scan(self, key: int) -> np.ndarray:
        if key not in self.keyed_scans:
            raise Exception(f"key {key} does not exist.")
        return np.asarray(self.keyed_scans[key].points)

    def get_scan_2d(self, key: int) -> np.ndarray:
        scan_3d = self.get_scan(key)
        return scan_3d[:, :2]

    def transform_scan(self, key: int, T: np.ndarray) -> None:
        if key not in self.keyed_scans:
            raise Exception(f"key {key} does not exist.")
        self.keyed_scans[key].transform(T)

    def clean_scans(self):
        for key in self.keyed_scans:
            self.keyed_scans[key], __ = self.keyed_scans[
                key
            ].remove_statistical_outlier(nb_neighbors=10, std_ratio=2.0)


def load_scans(directory: str) -> KeyedScans:
    scans = KeyedScans()
    for filename in os.listdir(directory):
        if filename.endswith(".pcd"):
            pcd_file = os.path.join(directory, filename)
            key = int(filename.split(".")[0])
            pcd = o3d.io.read_point_cloud(pcd_file)
            scans.add_scan(key, pcd)
    return scans
