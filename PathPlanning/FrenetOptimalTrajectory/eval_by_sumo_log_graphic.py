import argparse
import json
import numpy as np
import matplotlib.pyplot as plt
import math

class LogHandler:
    def __init__(self, lfp):
        self.data = []

        with open(lfp, "r") as f:
            self.data = [json.loads(l) for l in f.readlines()]

    def methods(self):
        return list(set([d["name"] for d in self.data]))

    def times(self):
        return sorted(list(set([float(d["time"]) for d in self.data])))

    def data_by_name(self, name):
        return [d for d in self.data if d["name"] == name]

    def attributes_by_name(self, attr, name):
        return [l[attr] for l in sorted(self.data_by_name(name), key=lambda k: k["time"])]

    def sizes(self, name):
        return self.attributes_by_name("size", name)

    def comp_times(self, name):
        return self.attributes_by_name("comp_time", name)

    def decomp_times(self, name):
        return self.attributes_by_name("decomp_time", name)

    def max_errors(self, name):
        return [d for d in self.attributes_by_name("max_error", name) if d != 0]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Dynamic Map.')
    parser.add_argument('-lfp', '--log_file_path')
    args = parser.parse_args()

    log_handler = LogHandler(args.log_file_path)

    times = log_handler.times()

    BINS = np.linspace(0, 1, 50)
    for name in log_handler.methods():
        tmp = [1 - (d / 800) for d in log_handler.sizes(name)]
        print(f"{name}, {min(tmp)}, {max(tmp)}, {np.percentile(tmp, [25, 50, 75])}")
        plt.hist(tmp, alpha=0.5, bins=BINS, label=name, rwidth=10)
    plt.legend(loc="upper left", fontsize=13)
    plt.show()

    BINS = np.linspace(-4.5, -0.5, 200)
    for name in log_handler.methods():
        if name == "esti":
            continue
        tmp = [math.log10(d) for d in log_handler.comp_times(name)]
        print(f"{name}, {min(tmp)}, {max(tmp)}, {np.percentile(tmp, [25, 50, 75])}")
        plt.hist(tmp, alpha=0.5, bins=BINS, label=name, rwidth=10)
    plt.legend(loc="upper left", fontsize=13)
    plt.show()

    BINS = np.linspace(-4.0, -2.0, 100)
    for name in log_handler.methods():
        if name == "esti":
            continue
        tmp = [math.log10(d) for d in log_handler.decomp_times(name)]
        print(f"{name}, {min(tmp)}, {max(tmp)}, {np.percentile(tmp, [25, 50, 75])}")
        plt.hist(tmp, alpha=0.5, bins=BINS, label=name, rwidth=10)
    plt.legend(loc="upper left", fontsize=13)
    plt.show()

    BINS = np.linspace(-3.0, 0, 100)
    for name in log_handler.methods():
        if name == "esti":
            continue
        tmp = [math.log10(d) for d in log_handler.max_errors(name)]
        print(f"{name}, {min(tmp)}, {max(tmp)}, {np.percentile(tmp, [25, 50, 75])}")
        plt.hist(tmp, alpha=0.5, bins=BINS, label=name, rwidth=10)
    plt.legend(loc="upper left", fontsize=13)
    plt.show()
