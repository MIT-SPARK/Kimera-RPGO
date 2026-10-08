#include <pybind11/eigen.h>

#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>
#include <sstream>
#include <string>

#include "kimera_rpgo/pcm/pcm.h"
#include "kimera_rpgo/rpgo.h"
#include "kimera_rpgo/utils/loss_functions.h"

namespace py = pybind11;
using namespace py::literals;

using namespace kimera_rpgo;

/**
 * Python interface with pybind11
 */
PYBIND11_MODULE(_kimera_rpgo_bindings, m) {
  m.doc() = "Python binding for RPGO-tools";

  // Python bound for RpgoConfig::SolverType
  py::enum_<RpgoConfig::SolverType>(m, "SolverType")
      .value("LEAST_SQUARES", RpgoConfig::SolverType::LEAST_SQUARES)
      .value("GRADIENT", RpgoConfig::SolverType::GRADIENT);

  // Python bound for LossType
  py::enum_<LossType>(m, "LossType")
      .value("LS", LossType::LS)
      .value("HUBER", LossType::HUBER)
      .value("GM", LossType::GM)
      .value("TUKEY", LossType::TUKEY)
      .value("TLS", LossType::TLS);

  // Python bound for SolverConfig::LeastSquaresOption
  py::enum_<SolverConfig::LeastSquaresOption>(m, "LeastSquaresOption")
      .value("GN", SolverConfig::LeastSquaresOption::GN)
      .value("LM", SolverConfig::LeastSquaresOption::LM)
      .value("DOGLEG", SolverConfig::LeastSquaresOption::DOGLEG);

  // Python bound for SolverConfig::GradientOption
  py::enum_<SolverConfig::GradientOption>(m, "GradientOption")
      .value("GRADIENT", SolverConfig::GradientOption::GRADIENT)
      .value("CONJUGATE", SolverConfig::GradientOption::CONJUGATE);

  // Python bound for OptimizerParams
  py::class_<OptimizerParams, std::shared_ptr<OptimizerParams>>(
      m, "OptimizerParams")
      .def_readwrite("maxIterations", &OptimizerParams::maxIterations)
      .def_readwrite("relativeErrorTol", &OptimizerParams::relativeErrorTol)
      .def_readwrite("absoluteErrorTol", &OptimizerParams::absoluteErrorTol)
      .def_readwrite("errorTol", &OptimizerParams::errorTol)
      .def("__repr__", [](const OptimizerParams& a) {
        std::stringstream stream;
        stream << a;
        return stream.str();
      });

  // Python bound for GaussNewtonParams
  py::class_<GaussNewtonParams,
             OptimizerParams,
             std::shared_ptr<GaussNewtonParams>>(m, "GaussNewtonParams")
      .def_readwrite("maxIterations", &OptimizerParams::maxIterations)
      .def_readwrite("relativeErrorTol", &OptimizerParams::relativeErrorTol)
      .def_readwrite("absoluteErrorTol", &OptimizerParams::absoluteErrorTol)
      .def_readwrite("errorTol", &OptimizerParams::errorTol)
      .def("__repr__", [](const OptimizerParams& a) {
        std::stringstream stream;
        stream << a;
        return stream.str();
      });

  // Python bound for LevenbergMarquardtParams
  py::class_<LevenbergMarquardtParams,
             OptimizerParams,
             std::shared_ptr<LevenbergMarquardtParams>>(
      m, "LevenbergMarquardtParams")
      .def_readwrite("maxIterations", &OptimizerParams::maxIterations)
      .def_readwrite("relativeErrorTol", &OptimizerParams::relativeErrorTol)
      .def_readwrite("absoluteErrorTol", &OptimizerParams::absoluteErrorTol)
      .def_readwrite("errorTol", &OptimizerParams::errorTol)
      .def("__repr__", [](const OptimizerParams& a) {
        std::stringstream stream;
        stream << a;
        return stream.str();
      });

  // Python bound for DoglegParams
  py::class_<DoglegParams, OptimizerParams, std::shared_ptr<DoglegParams>>(
      m, "DoglegParams")
      .def_readwrite("maxIterations", &OptimizerParams::maxIterations)
      .def_readwrite("relativeErrorTol", &OptimizerParams::relativeErrorTol)
      .def_readwrite("absoluteErrorTol", &OptimizerParams::absoluteErrorTol)
      .def_readwrite("errorTol", &OptimizerParams::errorTol)
      .def("__repr__", [](const OptimizerParams& a) {
        std::stringstream stream;
        stream << a;
        return stream.str();
      });

  // Python bound for GncParams
  py::class_<GncParams, GncParams::Ptr>(m, "GncParams")
      .def(py::init<>())
      .def_readwrite("barc_sq", &GncParams::barc_sq)
      .def_readwrite("inlier_probability", &GncParams::inlier_probability)
      .def_readwrite("mu_step", &GncParams::mu_step)
      .def_readwrite("max_iterations", &GncParams::max_iterations)
      .def_readwrite("fix_odom", &GncParams::fix_odom)
      .def_readwrite("robust_cost", &GncParams::robust_cost)
      .def("__repr__", [](const GncParams& a) {
        std::stringstream stream;
        stream << a;
        return stream.str();
      });

  // Python bound for SolverConfig
  py::class_<SolverConfig>(m, "SolverConfig")
      .def(py::init<>())
      .def_readwrite("verbosity", &SolverConfig::verbosity)
      .def_readwrite("least_squares_option",
                     &SolverConfig::least_squares_option)
      .def_readwrite("gradient_option", &SolverConfig::gradient_option)
      .def_readwrite("optimizer_params", &SolverConfig::optimizer_params)
      .def_readwrite("gnc_params", &SolverConfig::gnc_params)
      .def("setLeastSquaresParams", &SolverConfig::setLeastSquaresParams)
      .def("setLeastSquaresParamsDefault",
           &SolverConfig::setLeastSquaresParamsDefault)
      .def("setGradientParams", &SolverConfig::setGradientParams)
      .def("setGradientParamsDefault", &SolverConfig::setGradientParamsDefault)
      .def("setGncParamsDefault", &SolverConfig::setGncParamsDefault)
      .def("setGncParams", &SolverConfig::setGncParams)
      .def("__repr__", [](const SolverConfig& a) {
        std::stringstream stream;
        stream << a;
        return stream.str();
      });

  // Python bound for SolverConfig::GradientOption
  py::enum_<PcmConfig::MaxCliqueMode>(m, "PcmMaxCliqueMode")
      .value("EXACT", PcmConfig::MaxCliqueMode::EXACT)
      .value("HEURISTIC", PcmConfig::MaxCliqueMode::HEURISTIC)
      .value("INCREMENTAL", PcmConfig::MaxCliqueMode::INCREMENTAL);

  // Python bound for SolverConfig::GradientOption
  py::enum_<PcmConfig::MetricType>(m, "PcmMetricType")
      .value("NODE", PcmConfig::MetricType::NODE)
      .value("COVARIANCE", PcmConfig::MetricType::COVARIANCE);

  // Python bound fo PcmConfig
  py::class_<PcmConfig>(m, "PcmConfig")
      .def(py::init<>())
      .def_readwrite("odom_check", &PcmConfig::odom_check)
      .def_readwrite("trans_threshold", &PcmConfig::trans_threshold)
      .def_readwrite("rot_threshold", &PcmConfig::rot_threshold)
      .def_readwrite("mahalanobis_threshold", &PcmConfig::mahalanobis_threshold)
      .def_readwrite("odom_trans_threshold", &PcmConfig::odom_trans_threshold)
      .def_readwrite("odom_rot_threshold", &PcmConfig::odom_rot_threshold)
      .def_readwrite("odom_mahalanobis_threshold",
                     &PcmConfig::odom_mahalanobis_threshold)
      .def_readwrite("max_clique_mode", &PcmConfig::max_clique_mode)
      .def_readwrite("metric_type", &PcmConfig::metric_type)
      .def("__repr__", [](const PcmConfig& a) {
        std::stringstream stream;
        stream << a;
        return stream.str();
      });

  // Python bound for RpgoConfig
  py::class_<RpgoConfig>(m, "RpgoConfig")
      .def(py::init<>())
      .def_readwrite("solver_type", &RpgoConfig::solver_type)
      .def_readwrite("solver_config", &RpgoConfig::solver_config)
      .def_readwrite("pcm_config", &RpgoConfig::pcm_config)
      .def_readwrite("use_pcm", &RpgoConfig::use_pcm)
      .def_readwrite("loss_type", &RpgoConfig::loss_type)
      .def_readwrite("loss_threshold_c", &RpgoConfig::loss_threshold_c)
      .def("__repr__", [](const RpgoConfig& a) {
        std::stringstream stream;
        stream << a;
        return stream.str();
      });

  // Python bound for Rpgo
  py::class_<Rpgo>(m, "Rpgo")
      .def(py::init<RpgoConfig>())
      .def("loadG2o", &Rpgo::loadG2o)
      .def("fixFirstPose", &Rpgo::fixFirstPose)
      .def("setCorruptedOdom", &Rpgo::setCorruptedOdom)
      .def("run", &Rpgo::run)
      .def("writeResult", &Rpgo::writeResult)
      .def("writeLog", &Rpgo::writeLog);

  auto m_loss = m.def_submodule("robust_loss", "Robust loss functions.");

  py::class_<robust_loss::L2>(m_loss, "L2")
      .def(py::init())
      .def("weight", &robust_loss::L2::weight)
      .def("loss", &robust_loss::L2::loss);

  py::class_<robust_loss::TruncatedL2>(m_loss, "TruncatedL2")
      .def(py::init<double>(), py::arg("k") = 1.0)
      .def("weight", &robust_loss::TruncatedL2::weight)
      .def("loss", &robust_loss::TruncatedL2::loss);

  py::class_<robust_loss::Huber>(m_loss, "Huber")
      .def(py::init<double>(), py::arg("k") = 1.345)
      .def("weight", &robust_loss::Huber::weight)
      .def("loss", &robust_loss::Huber::loss);

  py::class_<robust_loss::Cauchy>(m_loss, "Cauchy")
      .def(py::init<double>(), py::arg("k") = 0.1)
      .def("weight", &robust_loss::Cauchy::weight)
      .def("loss", &robust_loss::Cauchy::loss);

  py::class_<robust_loss::Tukey>(m_loss, "Tukey")
      .def(py::init<double>(), py::arg("c") = 4.6851)
      .def("weight", &robust_loss::Tukey::weight)
      .def("loss", &robust_loss::Tukey::loss);

  py::class_<robust_loss::Welsch>(m_loss, "Welsch")
      .def(py::init<double>(), py::arg("c") = 2.9846)
      .def("weight", &robust_loss::Welsch::weight)
      .def("loss", &robust_loss::Welsch::loss);

  py::class_<robust_loss::GemanMcClure>(m_loss, "GemanMcClure")
      .def(py::init<double>(), py::arg("c") = 1.0)
      .def("weight", &robust_loss::GemanMcClure::weight)
      .def("loss", &robust_loss::GemanMcClure::loss);

  py::class_<robust_loss::MaxConsensus>(m_loss, "MaxConsensus")
      .def(py::init<double>(), py::arg("k") = 1.0)
      .def("weight", &robust_loss::MaxConsensus::weight)
      .def("loss", &robust_loss::MaxConsensus::loss);
}
