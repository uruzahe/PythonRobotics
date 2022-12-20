# -*- coding: utf-8 -*-
import sys
import pandas as pd
import numpy as np
from sklearn import linear_model
import joblib
from sklearn import preprocessing
from copy import deepcopy
import math
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import json

# from mpl_toolkits.mplot3d import Axes3D
# import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
# import numpy as np
# import matplotlib.pyplot as plt
# import numpy as np
# import numpy as np

# import matplotlib
# matplotlib.use('TkAgg')
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

def logistic(df, x_keys, y_keys):
    # print(f"all     data: {len(df)}")
    # df =
    # print(f"filterd data: {len(df)}")
    clf = linear_model.LinearRegression()

    X = df[x_keys]
    Y = df[y_keys]

    sscaler = preprocessing.StandardScaler()
    sscaler.fit(X)
    xss_sk = sscaler.transform(X)

    print(Y.isnull().sum())
    print(Y.isnull().all())
    # print(Y[265:270])
    # print(Y[Y["prop_size"].isnull()])
    # print(Y)
    # tmp = Y.dropna(how='all').dropna(how='all', axis=1)
    # print("tmp")
    # print(tmp)
    # print("tmp")
    clf.fit(xss_sk, Y)


    a = clf.coef_
    b = clf.intercept_

    # print(df["file_name"])
    # print(df.keys())
    print("回帰係数:", a) # 回帰係数: [ 0.70068905 -0.64667957]
    print("切片:", b) # 切片: 12.184694815481187
    # print("決定係数:", clf.score(X, Y)) # 決定係数: 0.6624246214760455



def three_d_fig(df, x_key, y_key, z_key, file_name):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    x = df[x_key]
    y = df[y_key]
    z = df[z_key]
    # ax.scatter(x, y, z)
    ax.set_xlabel(r"$I_{head}$ (sec)") # x軸ラベル
    ax.set_ylabel(r"$\Delta_{head} (deg)$") # y軸ラベル
    ax.set_zlabel("size (byte)") # z軸ラベル
    ax.set_zlim(0, 500)
    # ax.set_zscale('log')
    # ax.set_title('shade=True', fontsize='20') # タイトル
    # plt.show() # 描画
    colors=[]
    # z_logs = []
    for index, data in y.iteritems():
        # z_logs.append(data)
        if data == 0:
            colors.append("blue")

        elif data <= 45:
            colors.append("green")

        else:
            colors.append("orange")

    # print(z_logs)
    ax.bar(x, z, zs=y, zdir='y', color=colors, alpha=0.5)
    # fig.colorbar(p, ax=ax)
    # X, Y = np.meshgrid(x, y)
    # # 高度の計算式
    # Z = z
    # # 曲面を描画
    # ax.plot_surface(X, Y, Z, cmap = "summer")
    # # 底面に等高線を描画
    # ax.contour(X, Y, Z, colors = "black", offset = -1)
    plt.savefig(file_name, transparent=True)
    plt.close()


if __name__ == "__main__":
    data = []
    with open(sys.argv[1], "r") as f:
        lines = f.readlines()

        for l in lines:
            data.append(json.loads(l))

    df = pd.DataFrame.from_dict(data)
    # df = pd.read_csv(sys.argv[1], sep=',', skipinitialspace=True)
    print(df.keys())
    print(df)
    print(f"acc:                        {set(df['a_init'])}")
    print(f"speed:                      {set(df['v_init'])}")
    # print(f"dim:                        {set(df['dim'])}")
    print(f"rotate_update_interval:     {set(df['rotate_update_interval'])}")
    print(f"point:                      {set(df['np'])}")
    # print(f"d_dt:                       {set(df['d_dt'])}")
    print(f"angle_init:                 {set(df['t_init'])}")
    print(f"rotate_angle:               {set(df['rotate_value'])}")
    print(f"maxdim:                     {set(df['maxdim'])}")

    pd.set_option("display.max_colwidth", 1000)
    print(df.loc[[df['prop_size'].idxmax()]])
    print("\n----- logistic -----")
    xs_keys = ["a_init", "v_init", "rotate_update_interval", "np", "t_init", "rotate_value", "maxdim"]
    print(xs_keys)

    logistic(deepcopy(df), xs_keys, ['normal_size'])
    logistic(deepcopy(df), xs_keys, ['prop_size'])
    logistic(deepcopy(df), xs_keys, ['gzip_size'])
    logistic(deepcopy(df), xs_keys, ['esti_size'])
    logistic(deepcopy(df), xs_keys, ['huffman_size'])
    logistic(deepcopy(df), xs_keys, ['google_size'])
    logistic(deepcopy(df), xs_keys, ['all_size'])

    # データ準備
    prop_size = [d["prop_size"] for d in data]


    # グラフの描画
    # plt.hist([d["normal_size"] / d["normal_size"] for d in data if d["np"] == 100], alpha=0.5, bins=20, label="normal")
    NP = [100, 50]
    BINS = 40
    print(int(math.log2(len([d for d in data if d["np"] in NP]))))
    BINS = np.linspace(0, 1, 50)

    tmp = [1 - d["prop_size"]  / d["normal_size"] for d in data if d["np"] in NP]
    print(f"{min(tmp)}, {max(tmp)}, {np.percentile(tmp, [25, 50, 75])}")
    plt.hist(tmp, alpha=0.5, bins=BINS, label="prop", rwidth=10)

    tmp = [1 - d["gzip_size"]  / d["normal_size"] for d in data if d["np"] in NP]
    print(f"{min(tmp)}, {max(tmp)}, {np.percentile(tmp, [25, 50, 75])}")
    plt.hist(tmp, alpha=0.5, bins=BINS, label="prop", rwidth=10)

    tmp = [1 - d["esti_size"]  / d["normal_size"] for d in data if d["np"] in NP]
    print(f"{min(tmp)}, {max(tmp)}, {np.percentile(tmp, [25, 50, 75])}")
    plt.hist(tmp, alpha=0.5, bins=BINS, label="prop", rwidth=10)

    tmp = [1 - d["huffman_size"]  / d["normal_size"] for d in data if d["np"] in NP]
    print(f"{min(tmp)}, {max(tmp)}, {np.percentile(tmp, [25, 50, 75])}")
    plt.hist(tmp, alpha=0.5, bins=BINS, label="prop", rwidth=10)

    tmp = [1 - d["google_size"]  / d["normal_size"] for d in data if d["np"] in NP]
    print(f"{min(tmp)}, {max(tmp)}, {np.percentile(tmp, [25, 50, 75])}")
    plt.hist(tmp, alpha=0.5, bins=BINS, label="prop", rwidth=10)

    tmp = [1 - d["all_size"]  / d["normal_size"] for d in data if d["np"] in NP]
    print(f"{min(tmp)}, {max(tmp)}, {np.percentile(tmp, [25, 50, 75])}")
    plt.hist(tmp, alpha=0.5, bins=BINS, label="prop", rwidth=10)

    plt.legend(loc="upper left", fontsize=13) # (5)凡例表示
    plt.show()


    # logistic(deepcopy(df), xs_keys, ['normal_size'])
    logistic(deepcopy(df), xs_keys, ['prop_comp_time'])
    logistic(deepcopy(df), xs_keys, ['gzip_comp_time'])
    logistic(deepcopy(df), xs_keys, ['huffman_comp_time'])
    logistic(deepcopy(df), xs_keys, ['google_comp_time'])
    logistic(deepcopy(df), xs_keys, ['all_comp_time'])

    BINS = np.linspace(-4.5, -0.5, 200)

    tmp = [math.log10(d["prop_comp_time"]) for d in data if d["np"] in NP]
    print(f"{min(tmp)}, {max(tmp)}, {np.percentile(tmp, [25, 50, 75])}")
    plt.hist(tmp, alpha=0.5, bins=BINS, label="prop", rwidth=10)

    tmp = [math.log10(d["gzip_comp_time"]) for d in data if d["np"] in NP]
    print(f"{min(tmp)}, {max(tmp)}, {np.percentile(tmp, [25, 50, 75])}")
    plt.hist(tmp, alpha=0.5, bins=BINS, label="prop", rwidth=10)

    tmp = [math.log10(d["huffman_comp_time"]) for d in data if d["np"] in NP]
    print(f"{min(tmp)}, {max(tmp)}, {np.percentile(tmp, [25, 50, 75])}")
    plt.hist(tmp, alpha=0.5, bins=BINS, label="prop", rwidth=10)

    tmp = [math.log10(d["google_comp_time"]) for d in data if d["np"] in NP]
    print(f"{min(tmp)}, {max(tmp)}, {np.percentile(tmp, [25, 50, 75])}")
    plt.hist(tmp, alpha=0.5, bins=BINS, label="prop", rwidth=10)

    tmp = [math.log10(d["all_comp_time"]) for d in data if d["np"] in NP]
    print(f"{min(tmp)}, {max(tmp)}, {np.percentile(tmp, [25, 50, 75])}")
    plt.hist(tmp, alpha=0.5, bins=BINS, label="prop", rwidth=10)

    plt.legend(loc="upper left", fontsize=13) # (5)凡例表示
    plt.show()


    logistic(deepcopy(df), xs_keys, ['prop_decomp_time'])
    logistic(deepcopy(df), xs_keys, ['gzip_decomp_time'])
    logistic(deepcopy(df), xs_keys, ['huffman_decomp_time'])
    logistic(deepcopy(df), xs_keys, ['google_decomp_time'])
    logistic(deepcopy(df), xs_keys, ['all_decomp_time'])

    BINS = np.linspace(-4.0, -2.0, 100)
    tmp = [math.log10(d["prop_decomp_time"]) for d in data if d["np"] in NP]
    print(f"{min(tmp)}, {max(tmp)}, {np.percentile(tmp, [25, 50, 75])}")
    plt.hist(tmp, alpha=0.5, bins=BINS, label="prop", rwidth=10)

    tmp = [math.log10(d["gzip_decomp_time"]) for d in data if d["np"] in NP]
    print(f"{min(tmp)}, {max(tmp)}, {np.percentile(tmp, [25, 50, 75])}")
    plt.hist(tmp, alpha=0.5, bins=BINS, label="prop", rwidth=10)

    tmp = [math.log10(d["huffman_decomp_time"]) for d in data if d["np"] in NP]
    print(f"{min(tmp)}, {max(tmp)}, {np.percentile(tmp, [25, 50, 75])}")
    plt.hist(tmp, alpha=0.5, bins=BINS, label="prop", rwidth=10)

    tmp = [math.log10(d["google_decomp_time"]) for d in data if d["np"] in NP]
    print(f"{min(tmp)}, {max(tmp)}, {np.percentile(tmp, [25, 50, 75])}")
    plt.hist(tmp, alpha=0.5, bins=BINS, label="prop", rwidth=10)

    tmp = [math.log10(d["all_decomp_time"]) for d in data if d["np"] in NP]
    print(f"{min(tmp)}, {max(tmp)}, {np.percentile(tmp, [25, 50, 75])}")
    plt.hist(tmp, alpha=0.5, bins=BINS, label="prop", rwidth=10)

    plt.legend(loc="upper left", fontsize=13) # (5)凡例表示
    plt.show()

    logistic(deepcopy(df), xs_keys, ['prop_max_error'])
    logistic(deepcopy(df), xs_keys, ['gzip_max_error'])
    logistic(deepcopy(df), xs_keys, ['huffman_max_error'])
    logistic(deepcopy(df), xs_keys, ['google_max_error'])
    logistic(deepcopy(df), xs_keys, ['all_max_error'])

    BINS = np.linspace(-20.0, 0, 100)
    tmp = [math.log10(d["prop_max_error"]) for d in data if d["np"] in NP and d["prop_max_error"] != 0]
    print(f"{min(tmp)}, {max(tmp)}, {np.percentile(tmp, [25, 50, 75])}")
    plt.hist(tmp, alpha=0.5, bins=BINS, label="prop", rwidth=10)

    print([d["gzip_max_error"] for d in data if d["np"] in NP])
    tmp = [math.log10(d["gzip_max_error"]) for d in data if d["np"] in NP and d["gzip_max_error"] != 0]
    print(f"{min(tmp)}, {max(tmp)}, {np.percentile(tmp, [25, 50, 75])}")
    plt.hist(tmp, alpha=0.5, bins=BINS, label="prop", rwidth=10)

    tmp = [math.log10(d["huffman_max_error"]) for d in data if d["np"] in NP and d["huffman_max_error"] != 0]
    print(f"{min(tmp)}, {max(tmp)}, {np.percentile(tmp, [25, 50, 75])}")
    plt.hist(tmp, alpha=0.5, bins=BINS, label="prop", rwidth=10)

    tmp = [math.log10(d["google_max_error"]) for d in data if d["np"] in NP and d["google_max_error"] != 0]
    print(f"{min(tmp)}, {max(tmp)}, {np.percentile(tmp, [25, 50, 75])}")
    plt.hist(tmp, alpha=0.5, bins=BINS, label="prop", rwidth=10)

    tmp = [math.log10(d["all_max_error"]) for d in data if d["np"] in NP and d["all_max_error"] != 0]
    print(f"{min(tmp)}, {max(tmp)}, {np.percentile(tmp, [25, 50, 75])}")
    plt.hist(tmp, alpha=0.5, bins=BINS, label="prop", rwidth=10)

    plt.legend(loc="upper left", fontsize=13) # (5)凡例表示
    plt.show()
