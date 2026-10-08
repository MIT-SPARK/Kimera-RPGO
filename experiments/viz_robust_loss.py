import argparse
import os

import kimera_rpgo
import matplotlib.pyplot as plt
import numpy as np


def plot_loss(ax, loss_func, resolution=0.1, min_limit=-5, max_limit=5):
    distance = np.arange(min_limit, max_limit, resolution)
    get_loss = np.vectorize(loss_func.loss)
    loss = get_loss(distance)

    ax.plot(distance, loss)


def plot_influence(ax, loss_func, resolution=0.1, min_limit=-5, max_limit=5):
    distance = np.arange(min_limit, max_limit, resolution)
    get_weight = np.vectorize(loss_func.weight)
    weight = get_weight(distance)
    influence = np.multiply(weight, distance)

    ax.plot(distance, influence)


def plot_weight(ax, loss_func, resolution=0.1, min_limit=-5, max_limit=5):
    distance = np.arange(min_limit, max_limit, resolution)
    get_weight = np.vectorize(loss_func.weight)
    weight = get_weight(distance)

    ax.plot(distance, weight)


def get_loss_function(loss_str: str):
    if loss_str == "quadratic":
        return kimera_rpgo.robust_loss.L2()
    if loss_str == "tls":
        return kimera_rpgo.robust_loss.TruncatedL2()
    if loss_str == "huber":
        return kimera_rpgo.robust_loss.Huber()
    if loss_str == "cauchy":
        return kimera_rpgo.robust_loss.Cauchy()
    if loss_str == "tukey":
        return kimera_rpgo.robust_loss.Tukey()
    if loss_str == "welsch":
        return kimera_rpgo.robust_loss.Welsch()
    if loss_str == "gm":
        return kimera_rpgo.robust_loss.GemanMcClure()
    if loss_str == "max_consensus":
        return kimera_rpgo.robust_loss.MaxConsensus()


def main():
    parser = argparse.ArgumentParser(description="Visualize Robust Loss")
    parser.add_argument("output", help="Experiment config yaml")
    args = parser.parse_args()

    loss_to_plot = [
        "quadratic",
        "huber",
        "cauchy",
        "tukey",
        "welsch",
        "gm",
        "tls",
        "max_consensus",
    ]
    loss_title = [
        "Quadratic",
        "Huber",
        "Cauchy",
        "Tukey",
        "Welsch",
        "Geman-McClure",
        "Truncated-Quadratic",
        "Max-Consensus",
    ]
    rows = 4
    cols = 4

    save_separate = True
    fig = plt.figure()
    idx = 1
    for name, loss in zip(loss_title, loss_to_plot):
        lf = get_loss_function(loss)
        if save_separate:
            ax = fig.add_subplot(1, 1, 1)
        else:
            ax = fig.add_subplot(rows, cols, idx)
        plot_loss(ax, lf)
        ax.axis("off")
        if save_separate:
            plt.savefig(os.path.join(args.output, f"{name}_loss.png"))
            fig = plt.figure()
        else:
            ax.set_title(f"{name} loss")
        idx += 1

        if save_separate:
            ax = fig.add_subplot(1, 1, 1)
        else:
            ax = fig.add_subplot(rows, cols, idx)
        plot_influence(ax, lf)
        ax.axis("off")
        if save_separate:
            plt.savefig(os.path.join(args.output, f"{name}_influence.png"))
            fig = plt.figure()
        else:
            ax.set_title(f"{name} influence")
        idx += 1

    if not save_separate:
        plt.show()


if __name__ == "__main__":
    main()
