import sys
import re
import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':


    if len(sys.argv) < 2:
        print("Usage: python avg_color.py <input_file>")
        sys.exit(1)
    input_file = sys.argv[1]
    # input_file = "dsd_off_1min.log"

    pattern_color = re.compile(r"^- color \(([^)]*)\)")
    pattern_weight = re.compile(r"^- dstReservoir\.weight\s+([+-]?\d*\.?\d+(?:[eE][+-]?\d+)?)")

    sum_x = sum_y = sum_z = 0.0
    count = 0

    color_x = []
    color_y = []
    color_z = []
    weights = []

    with open(input_file, "r", encoding="utf8") as f:
        for line in f:

            match_color = pattern_color.match(line)
            if match_color:
                # Extract the x,y,z part inside the parentheses
                values = match_color.group(1).split(",")
                x, y, z = [float(v.strip()) for v in values]
                color_x.append(x)
                color_y.append(y)
                color_z.append(z)
                continue

            # Extract weights
            match_weight = pattern_weight.match(line)
            if match_weight:
                w = float(match_weight.group(1))
                weights.append(w)
                continue


    weights = np.array([weights])
    weights[weights > 10] = 10


    avg_color = np.array([ np.sum(color_x), np.sum(color_y), np.sum(color_z) ])
    len_color = len(color_x)
    avg_color /= len_color
    print(f"Avg color: {avg_color}")
    print(f"num colors {len_color}")

    plt.figure()
    plt.hist(color_x, bins= 100, color= 'r', alpha= 0.5, label= 'R')
    plt.hist(color_y, bins= 100, color= 'g', alpha= 0.5, label= 'G')
    plt.hist(color_z, bins= 100, color= 'b', alpha= 0.5, label= 'B')
    plt.xlim(0, 3)
    plt.legend()
    plt.title("Color Hist")
    # plt.show()


    num_weights = weights.shape[1]
    avg_weight = np.sum(weights) / num_weights
    print(f"Avg weight: {avg_weight}")
    print(f"num weights: {num_weights}")

    plt.figure()
    plt.hist(weights.tolist(), bins= 100, alpha= 0.5, label= 'weight')
    plt.xlim(0, 10)
    plt.legend()
    plt.title("Reservoir weight Hist")
    plt.show()
