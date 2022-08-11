/*
Outlier removal class
Provide a set of outlier removal methods
along with interface to KimeraRPGO
author: Yun Chang
*/

#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <fstream>
#include <string>

#include "KimeraRPGO/utils/TypeUtils.h"

namespace KimeraRPGO {

class OutlierRemoval {
 public:
  OutlierRemoval() = default;
  virtual ~OutlierRemoval() = default;

  virtual size_t getNumLC() = 0;
  virtual size_t getNumLCInliers() = 0;
  virtual size_t getNumOdomFactors() = 0;
  virtual size_t getNumSpecialFactors() = 0;

  /*! \brief Process new measurements and reject outliers
   *  process the new measurements and update the "good set" of measurements
   *  - new_factors: factors from the new measurements
   *  - new_values: linearization point of the new measurements
   *	- nfg: the factors after processing new measurements and outlier removal
   * 	- values: the values after processing new measurements and outlier
   *removal
   *  - returns: boolean of if optimization should be called or not
   */
  virtual bool removeOutliers(const gtsam::NonlinearFactorGraph& new_factors,
                              const gtsam::Values& new_values,
                              gtsam::NonlinearFactorGraph* nfg,
                              gtsam::Values* values) = 0;

  /*! \brief Save any data in the outlier removal process
   *  - folder_path: path to directory to save results in
   */
  virtual void saveData(std::string folder_path) {}

  /*! \brief Supressing the print messages to console
   */
  void setQuiet() { debug_ = false; }

  /*! \brief Set log folder
   */
  void logOutput(const std::string& output_folder) {
    log_output_ = true;
    log_folder_ = output_folder;
    std::string filename = output_folder + "/outlier_rejection_status.txt";
    std::ofstream outfile;
    outfile.open(filename);
    outfile << "total inliers spin-time mc-time\n";
    outfile.close();
  }

  /*! \brief Remove last measured loop closure
   */
  virtual EdgePtr removeLastLoopClosure(
      ObservationId id,
      gtsam::NonlinearFactorGraph* updated_factors) { return nullptr; }

  /*! \brief Remove last measured loop closure regardless of obs id
   */
  virtual EdgePtr removeLastLoopClosure(
      gtsam::NonlinearFactorGraph* updated_factors) { return nullptr; }

  /*! \brief Ignore all loop closures that involves certain prefix
   */
  virtual void ignoreLoopClosureWithPrefix(
      char prefix,
      gtsam::NonlinearFactorGraph* updated_factors) {}

  /*! \brief add back all loop closures that involves certain prefix
   * Basically undos ignore
   */
  virtual void reviveLoopClosureWithPrefix(
      char prefix,
      gtsam::NonlinearFactorGraph* updated_factors) {}

  /*! \brief Get the vector of currently ignored prefixes
   */
  virtual inline std::vector<char> getIgnoredPrefixes() { return {}; }

  /*! \brief Remove prior factors of nodes with prefix prefix
   */
  virtual void removePriorFactorsWithPrefix(
      const char& prefix,
      gtsam::NonlinearFactorGraph* updated_factors) {}

 protected:
  bool debug_ = true;
  bool log_output_ = false;
  std::string log_folder_;
};

}  // namespace KimeraRPGO
