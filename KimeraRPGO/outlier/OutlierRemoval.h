/*
Outlier removal class
Provide a set of outlier removal methods
along with interface to KimeraRPGO
author: Yun Chang
*/

#ifndef KIMERARPGO_OUTLIER_OUTLIERREMOVAL_H_
#define KIMERARPGO_OUTLIER_OUTLIERREMOVAL_H_

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <string>

#include "KimeraRPGO/utils/type_utils.h"

namespace KimeraRPGO {

class OutlierRemoval {
 public:
  OutlierRemoval() = default;
  virtual ~OutlierRemoval() = default;

  virtual size_t getNumLC() {}
  virtual size_t getNumLCInliers() {}

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
                              gtsam::NonlinearFactorGraph& nfg,
                              gtsam::Values& values) = 0;

  /*! \brief Save any data in the outlier removal process
   *  - folder_path: path to directory to save results in
   */
  virtual void saveData(std::string folder_path) {}

  /*! \brief Supressing the print messages to console
   */
  void setQuiet() { debug_ = false; }

  /*! \brief Remove last measured loop closure
   */
  virtual void removeLastLoopClosure(
      ObservationId id,
      gtsam::NonlinearFactorGraph* updated_factors) {}

 protected:
  bool debug_ = true;
};

}  // namespace KimeraRPGO
#endif  // KIMERARPGO_OUTLIER_OUTLIERREMOVAL_H_
