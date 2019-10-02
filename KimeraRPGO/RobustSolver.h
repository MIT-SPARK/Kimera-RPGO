/*
Backend solver class (Robust Pose Graph Optimizer)
author: Yun Chang, Luca Carlone
*/

#ifndef KIMERARPGO_ROBUSTSOLVER_H_
#define KIMERARPGO_ROBUSTSOLVER_H_

#include <memory>
#include <string>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "KimeraRPGO/GenericSolver.h"
#include "KimeraRPGO/SolverParams.h"
#include "KimeraRPGO/outlier/OutlierRemoval.h"

namespace KimeraRPGO {

/*! \brief RobustSolver type.
 *  Main backend solver that can do outlier rejection
 *  - params: RobustSolverParams, accounts for the outlier rejection method,
 * etc. (see SolverParams.h for details on RobustSolverParams)
 */
class RobustSolver : public GenericSolver {
 public:
  explicit RobustSolver(const RobustSolverParams& params);

  virtual ~RobustSolver() = default;

  size_t getNumLC() { return outlier_removal_->getNumLC(); }

  size_t getNumLCInliers() { return outlier_removal_->getNumLCInliers(); }

  /*! \brief Normal update call for Robust Solver
   *  add new factors and values and optimize, possibly after rejecting
   * outliers.
   *  - nfg: new factors
   *  - values: linearization point for new variables
   */
  void update(
      const gtsam::NonlinearFactorGraph& nfg = gtsam::NonlinearFactorGraph(),
      const gtsam::Values& values = gtsam::Values());

  /*! \brief Update call that bypasses outlier rejection.
   *  add new factors and values and optimize, without rejecting outliers.
   *  - nfg: new factors
   *  - values: linearization point for new variables
   */
  void forceUpdate(
      const gtsam::NonlinearFactorGraph& nfg = gtsam::NonlinearFactorGraph(),
      const gtsam::Values& values = gtsam::Values());

  /*! \brief add an odometry edge
   * odom_factor: a gtsam::NonlinearFactorGraph with a single factor
   * odom_values: a gtsam::Values with a single value
   */
  void addOdometry(const gtsam::NonlinearFactorGraph& odom_factor,
                   const gtsam::Values& odom_values);

  /*! \brief Used with Add and Load to connect factor graph to between or prior
   * factor Sorts through the factors, separate out the odometry, the landmark
   * measurements, and loop closures, then addAndCheckIfOptimize with outlier
   * rejection
   *  - factors: the factors of the graph to be added
   *  - values: linearization point of graph to be connected
   *  - key0: Lowest key of the graph to be connected (root of odometry)
   */
  void updateBatch(const gtsam::NonlinearFactorGraph& factors,
                   const gtsam::Values& values,
                   const gtsam::Key& key0);

 private:
  std::unique_ptr<OutlierRemoval> outlier_removal_;  // outlier removal method;

  /*! \brief Calling the optimization
   *  Optimize the factor graph with the stroed values
   *  Solver based on what was set in RobustSolverParams
   */
  void optimize();

 public:
  /*! \brief Save results from Solver
   *  Saves the resulting g2o file and also the data saved in the outlier
   * removal method.
   *  - folder_path: the directory to save the results.
   */
  void saveData(std::string folder_path) const;

  /*! \brief Load a factor graph with a prior
   *  NOTE: PROJECT SPECIFIC MAY INCLUDED SPECIFIC ASSUMPTIONS
   *  Assuming a newly initialized RobustSolver. Use load graph to load a whole
   * factor graph
   *  - factors: the nonlinear factor graph to be loaded
   *  - values: linearization point for the loaded variables
   *  - prior: the prior factor anchoring the graph, associated with the first
   * value (lowest key ex. 0, or a0, etc. )
   */
  template <class T>
  void loadGraph(const gtsam::NonlinearFactorGraph& factors,
                 const gtsam::Values& values,
                 const gtsam::PriorFactor<T>& prior) {
    gtsam::NonlinearFactorGraph prior_factor;
    gtsam::Values prior_values;
    prior_factor.add(prior);
    prior_values.insert(prior.key(), prior.prior());

    addOdometry(prior_factor, prior_values);
    updateBatch(factors, values, prior.key());
  }

  /*! \brief Load a factor graph without a prior
   *  NOTE: PROJECT SPECIFIC MAY INCLUDED SPECIFIC ASSUMPTIONS
   *  Assuming a newly initialized RobustSolver. Use load graph to load a whole
   * factor graph
   *  - factors: the nonlinear factor graph to be loaded
   *  - values: linearization point for the loaded variables
   *  - key0: the key associated with the first value (lowest key ex. 0, or a0,
   * etc. )
   */
  template <class T>
  void loadGraph(gtsam::NonlinearFactorGraph factors,
                 gtsam::Values values,
                 gtsam::Key key0 = 0) {
    gtsam::Values prior_values;
    prior_values.insert(key0, values.at<T>(key0));

    addOdometry(gtsam::NonlinearFactorGraph(), prior_values);
    updateBatch(factors, values, key0);
  }

  /*! \brief Add a factor graph to solver with a between factor for connection
   *  NOTE: PROJECT SPECIFIC MAY INCLUDED SPECIFIC ASSUMPTIONS
   *  Assuming a RobustSolver that already has some existing information. Use
   * add graph to add another whole factor graph to the existing.
   *  - factors: the nonlinear factor graph to be added
   *  - values: linearization point for the added variables
   *  - connector: the BetweenFactor associated that attaches the new graph to
   * some point in the exisitng graph.
   */
  template <class T>
  void addGraph(const gtsam::NonlinearFactorGraph& factors,
                const gtsam::Values& values,
                const gtsam::BetweenFactor<T>& connector) {
    gtsam::Key key0 = connector.back();
    gtsam::NonlinearFactorGraph connect_factor;
    gtsam::Values connect_values;
    connect_factor.add(connector);
    connect_values.insert(key0, values.at<T>(key0));

    addOdometry(connect_factor, connect_values);
    updateBatch(factors, values, key0);
  }
};

}  // namespace KimeraRPGO

#endif  // KIMERARPGO_ROBUSTSOLVER_H_
