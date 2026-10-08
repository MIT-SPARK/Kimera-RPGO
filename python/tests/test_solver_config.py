"""Regression tests for parameter ownership across the Python/C++ boundary."""

import sys
import unittest

sys.path.insert(0, sys.argv.pop(1))
import _kimera_rpgo_bindings as rpgo  # noqa: E402


class SolverConfigOwnership(unittest.TestCase):
    def test_optimizer_parameters(self):
        for option in (
            rpgo.LeastSquaresOption.GN,
            rpgo.LeastSquaresOption.LM,
            rpgo.LeastSquaresOption.DOGLEG,
            None,
        ):
            with self.subTest(option=option):
                config = rpgo.SolverConfig()
                if option is None:
                    config.setGradientParamsDefault()
                else:
                    config.least_squares_option = option
                    config.setLeastSquaresParamsDefault()

                # A temporary Python wrapper must not delete C++-owned parameters.
                config.optimizer_params.maxIterations = 37
                self.assertEqual(config.optimizer_params.maxIterations, 37)
                params = config.optimizer_params
                config.optimizer_params = None
                del config
                self.assertEqual(params.maxIterations, 37)

                target = rpgo.SolverConfig()
                if option is None:
                    target.setGradientParams(params)
                else:
                    target.setLeastSquaresParams(params)
                    self.assertEqual(target.least_squares_option, option)

                target.optimizer_params = params
                del params
                self.assertEqual(target.optimizer_params.maxIterations, 37)

    def test_gnc_parameters(self):
        config = rpgo.SolverConfig()
        config.setGncParamsDefault()
        config.gnc_params.max_iterations = 43
        self.assertEqual(config.gnc_params.max_iterations, 43)
        params = config.gnc_params
        config.setGncParamsDefault()
        del config
        self.assertEqual(params.max_iterations, 43)

        config = rpgo.SolverConfig()
        config.gnc_params = params
        del params
        self.assertEqual(config.gnc_params.max_iterations, 43)

        params = rpgo.GncParams()
        params.max_iterations = 59
        config.setGncParams(params)
        del params
        self.assertEqual(config.gnc_params.max_iterations, 59)


if __name__ == "__main__":
    unittest.main()
