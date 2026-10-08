"""Regression tests for parameter ownership across the Python/C++ boundary."""

import _kimera_rpgo_bindings as rpgo
import pytest


@pytest.mark.parametrize(
    "option",
    [
        rpgo.LeastSquaresOption.GN,
        rpgo.LeastSquaresOption.LM,
        rpgo.LeastSquaresOption.DOGLEG,
        None,
    ],
    ids=["gauss_newton", "levenberg_marquardt", "dogleg", "gradient"],
)
def test_optimizer_parameters(option):
    config = rpgo.SolverConfig()
    if option is None:
        config.setGradientParamsDefault()
    else:
        config.least_squares_option = option
        config.setLeastSquaresParamsDefault()

    # A temporary Python wrapper must not delete C++-owned parameters.
    config.optimizer_params.maxIterations = 37
    assert config.optimizer_params.maxIterations == 37
    params = config.optimizer_params
    config.optimizer_params = None
    del config
    assert params.maxIterations == 37

    target = rpgo.SolverConfig()
    if option is None:
        target.setGradientParams(params)
    else:
        target.setLeastSquaresParams(params)
        assert target.least_squares_option == option

    target.optimizer_params = params
    del params
    assert target.optimizer_params.maxIterations == 37


def test_gnc_parameters():
    config = rpgo.SolverConfig()
    config.setGncParamsDefault()
    config.gnc_params.max_iterations = 43
    assert config.gnc_params.max_iterations == 43
    params = config.gnc_params
    config.setGncParamsDefault()
    del config
    assert params.max_iterations == 43

    config = rpgo.SolverConfig()
    config.gnc_params = params
    del params
    assert config.gnc_params.max_iterations == 43

    params = rpgo.GncParams()
    params.max_iterations = 59
    config.setGncParams(params)
    del params
    assert config.gnc_params.max_iterations == 59
