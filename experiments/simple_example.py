import argparse

import kimera_rpgo


def main():
    parser = argparse.ArgumentParser(description="Simple RPGO usage Example")
    parser.add_argument("g2o", help="input g2o file")
    parser.add_argument("output_g2o", help="output g2o file")
    parser.add_argument("--is_3d", action="store_true")
    parser.add_argument("--is_2d", dest="is_3d", action="store_false")
    parser.add_argument("--verbosity", "-v", default=1, type=int)
    parser.add_argument("--gradient_descent", "-gd", action="store_true")
    parser.add_argument(
        "--least_squares", "-nls", dest="gradient_descent", action="store_false"
    )
    parser.add_argument(
        "--robust_cost",
        "-r",
        default="none",
        type=str,
        help="none, huber, gm, tukey, tls",
    )
    parser.add_argument(
        "--gnc_alpha",
        "-g",
        default=1.0,
        type=float,
        help="GNC alpha",
    )
    parser.set_defaults(gradient_descent=False, is_3d=False)
    args = parser.parse_args()

    # Set up config
    config = kimera_rpgo.RpgoConfig()

    if args.gradient_descent:
        config.solver_type = kimera_rpgo.SolverType.GRADIENT
        config.solver_config.loadDefaultGradientConfig()
    else:
        config.solver_type = kimera_rpgo.SolverType.LEAST_SQUARES
        config.solver_config.loadDefaultLeastSquaresConfig()

    if args.gnc_alpha < 1.0 and args.gnc_alpha > 0.0:
        config.use_gnc = True
        config.solver_config.gnc_params.inlier_probability = args.gnc_alpha

    config.solver_config.verbosity = args.verbosity

    if args.robust_cost == "huber":
        config.loss_type = kimera_rpgo.LossType.HUBER
    elif args.robust_cost == "gm":
        config.loss_type = kimera_rpgo.LossType.GM
    elif args.robust_cost == "tukey":
        config.loss_type = kimera_rpgo.LossType.TUKEY
    elif args.robust_cost == "tls":
        config.loss_type = kimera_rpgo.LossType.TLS

    # Initialize RPGO solver
    test = kimera_rpgo.Rpgo(config)

    # Load g2o file
    test.loadG2o(args.g2o, args.is_3d)

    # Run optimizer
    test.run()

    # Save output
    test.writeResult(args.output_g2o)


if __name__ == "__main__":
    main()
