#pragma once

#include <gtsam/base/Testable.h>
#include <gtsam/base/types.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/sfm/BinaryMeasurement.h>
#include <gtsam/sfm/SfmData.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_unstable/slam/PoseToPointFactor.h>

#include <cmath>
#include <ctime>
#include <fstream>
#include <iosfwd>
#include <map>
#include <optional>
#include <string>
#include <utility>  // for pair
#include <vector>

#include "kimera_rpgo/utils/types.h"

namespace gtsam {

/// Robust kernel type to wrap around quadratic noise model
enum KernelFunctionType {
  KernelFunctionTypeNONE,
  KernelFunctionTypeHUBER,
  KernelFunctionTypeTUKEY,
  KernelFunctionTypeGM,
  KernelFunctionTypeTLS
};

/**
 * Parse variables in a line-based text format (like g2o) into a map.
 * Instantiated in .cpp Pose2, Point2, Pose3, and Point3.
 * Note the map keys are integer indices, *not* gtsam::Keys. This is is
 * different below where landmarks will use L(index) symbols.
 */
template <typename T>
std::map<size_t, T> parseVariables(const std::string& filename,
                                   size_t maxIndex = 0);

/**
 * Parse binary measurements in a line-based text format (like g2o) into a
 * vector. Instantiated in .cpp for Pose2, Rot2, Pose3, and Rot3. The rotation
 * versions parse poses and extract only the rotation part, using the marginal
 * covariance as noise model.
 */
template <typename T>
std::vector<BinaryMeasurement<T>> parseMeasurements(
    const std::string& filename,
    const noiseModel::Diagonal::shared_ptr& model = nullptr,
    size_t maxIndex = 0);

/// Return type for auxiliary functions
typedef std::pair<size_t, Pose2> IndexedPose;
typedef std::pair<size_t, Point2> IndexedLandmark;
typedef std::pair<std::pair<size_t, size_t>, Pose2> IndexedEdge;

/**
 * Parse TORO/G2O vertex "id x y yaw"
 * @param is input stream
 * @param tag string parsed from input stream, will only parse if vertex type
 */
std::optional<IndexedPose> parseVertexPose(std::istream& is,
                                           const std::string& tag);

/**
 * Parse G2O landmark vertex "id x y"
 * @param is input stream
 * @param tag string parsed from input stream, will only parse if vertex type
 */
std::optional<IndexedLandmark> parseVertexLandmark(std::istream& is,
                                                   const std::string& tag);

/**
 * Parse TORO/G2O edge "id1 id2 x y yaw"
 * @param is input stream
 * @param tag string parsed from input stream, will only parse if edge type
 */
std::optional<IndexedEdge> parseEdge(std::istream& is, const std::string& tag);

/// Return type for load functions, which return a graph and initial values. For
/// landmarks, the gtsam::Symbol L(index) is used to insert into the Values.
/// Bearing-range measurements also refer to landmarks with L(index).
using GraphAndValues =
    std::pair<NonlinearFactorGraph::shared_ptr, Values::shared_ptr>;

/**
 * Load TORO 2D Graph
 * @param dataset/model pair as constructed by [dataset]
 * @param maxIndex if non-zero cut out vertices >= maxIndex
 * @param addNoise add noise to the edges
 * @param smart try to reduce complexity of covariance to cheapest model
 */
GraphAndValues load2D(
    std::pair<std::string, SharedNoiseModel> dataset,
    size_t maxIndex = 0,
    bool addNoise = false,
    bool smart = true,  //
    KernelFunctionType kernelFunctionType = KernelFunctionTypeNONE);

/**
 * Load TORO/G2O style graph files
 * @param filename
 * @param model optional noise model to use instead of one specified by file
 * @param maxIndex if non-zero cut out vertices >= maxIndex
 * @param addNoise add noise to the edges
 * @param smart try to reduce complexity of covariance to cheapest model
 * @param noiseFormat how noise parameters are stored
 * @param kernelFunctionType whether to wrap the noise model in a robust kernel
 * @return graph and initial values
 */
GraphAndValues load2D(
    const std::string& filename,
    SharedNoiseModel model = SharedNoiseModel(),
    size_t maxIndex = 0,
    bool addNoise = false,
    bool smart = true,
    KernelFunctionType kernelFunctionType = KernelFunctionTypeNONE);

/** save 2d graph */
void save2D(const NonlinearFactorGraph& graph,
            const Values& config,
            const noiseModel::Diagonal::shared_ptr model,
            const std::string& filename);

/**
 * @brief This function parses a g2o file and stores the measurements into a
 * NonlinearFactorGraph and the initial guess in a Values structure
 * @param filename The name of the g2o file\
 * @param is3D indicates if the file describes a 2D or 3D problem
 * @param kernelFunctionType whether to wrap the noise model in a robust kernel
 * @return graph and initial values
 */
GraphAndValues readG2o(
    const std::string& g2oFile,
    const bool is3D = false,
    KernelFunctionType kernelFunctionType = KernelFunctionTypeNONE);

/**
 * @brief This function writes a g2o file from
 * NonlinearFactorGraph and a Values structure
 * @param filename The name of the g2o file to write
 * @param graph NonlinearFactor graph storing the measurements
 * @param estimate Values
 *
 * Note:behavior change in PR #471: to be consistent with load2D and load3D, we
 * write the *indices* to file and not the full Keys. This change really only
 * affects landmarks, which get read as indices but stored in values with the
 * symbol L(index).
 */
void writeG2o(const NonlinearFactorGraph& graph,
              const Values& estimate,
              const std::string& filename);

/// Load TORO 3D Graph
GraphAndValues load3D(
    const std::string& filename,
    KernelFunctionType kernelFunctionType = KernelFunctionTypeNONE);

using BinaryMeasurementsUnit3 = std::vector<BinaryMeasurement<Unit3>>;
using BinaryMeasurementsPoint3 = std::vector<BinaryMeasurement<Point3>>;
using BinaryMeasurementsRot3 = std::vector<BinaryMeasurement<Rot3>>;

KernelFunctionType gtsamLossType(const kimera_rpgo::LossType& loss_type);

SharedNoiseModel createRobustNoiseModel(SharedNoiseModel model,
                                        KernelFunctionType kernalFunctionType,
                                        double c);

GraphAndValues readG2owithLmks(const std::string& g2oFile,
                               bool is3D,
                               kimera_rpgo::LossType loss_type,
                               double robust_threshold_c);
}  // namespace gtsam
