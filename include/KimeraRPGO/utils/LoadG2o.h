#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam_unstable/slam/PoseToPointFactor.h>

#include <cmath>
#include <ctime>
#include <fstream>

namespace gtsam {

gtsam::GraphAndValues readG2owithLmks(const std::string& g2oFile,
                                      const bool is3D) {
  gtsam::GraphAndValues factors_values = readG2o(g2oFile, is3D);

  // reading file for landmark estimates and landmark edges (gtsam4.0.3
  // slam/dataset.cpp)
  std::ifstream is(g2oFile.c_str());
  if (!is) throw std::invalid_argument("cannot find file " + g2oFile);
  std::string tag;

  Key id1, id2;
  while (!is.eof()) {
    if (!(is >> tag)) {
      break;
    }

    // add 2d ldmk pose2point factor (if any)
    if (tag == "EDGE_SE2_XY") {
      double lmx, lmy;
      double v1, v2, v3;

      is >> id1 >> id2 >> lmx >> lmy >> v1 >> v2 >> v3;

      // Create noise model
      Matrix2 info_mat;
      info_mat << v1, v2, v2, v3;
      noiseModel::Gaussian::shared_ptr measurementNoise =
          noiseModel::Gaussian::Information(info_mat, true);  // smart = true

      // Add to graph
      *factors_values.first += gtsam::PoseToPointFactor<Pose2, Point2>(
          id1, id2, Point2(lmx, lmy), measurementNoise);
    }

    // add 3d ldmk pose2point factor (if any)
    if (tag == "EDGE_SE3_XYZ") {
      double lmx, lmy, lmz;
      double v11, v12, v13, v22, v23, v33;

      is >> id1 >> id2 >> lmx >> lmy >> lmz >> v11 >> v12 >> v13 >> v22 >>
          v23 >> v33;

      // Create noise model
      Matrix3 info_mat;
      info_mat << v11, v12, v13, v12, v22, v23, v13, v23, v33;
      noiseModel::Gaussian::shared_ptr measurementNoise =
          noiseModel::Gaussian::Information(info_mat, true);

      // Add to graph
      *factors_values.first += gtsam::PoseToPointFactor<Pose3, Point3>(
          id1, id2, Point3(lmx, lmy, lmz), measurementNoise);
    }
  }
  is.clear();
  is.seekg(0, std::ios::beg);  // guess back to beginning

  return factors_values;
}

}  // namespace gtsam