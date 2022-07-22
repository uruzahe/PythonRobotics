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

# from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
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

    # print(xss_sk)
    clf.fit(xss_sk, Y)

    a = clf.coef_
    b = clf.intercept_

    print(df["file_name"])
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
    ax.set_xlabel(x_key) # x軸ラベル
    ax.set_ylabel(y_key) # y軸ラベル
    ax.set_zlabel(z_key) # z軸ラベル
    # ax.set_zscale('log')
    # ax.set_title('shade=True', fontsize='20') # タイトル
    # plt.show() # 描画
    # colors=[]
    # z_logs = []
    # for index, data in z.iteritems():
    #     z_logs.append(data)
    #     if data <= 0.2:
    #         colors.append("orange")
    #
    #     elif data <= 0.4:
    #         colors.append("green")
    #
    #     else:
    #         colors.append("red")

    # print(z_logs)
    ax.bar(x, z, zs=y, zdir='y', alpha=0.5)
    # fig.colorbar(p, ax=ax)
    # X, Y = np.meshgrid(x, y)
    # # 高度の計算式
    # Z = z
    # # 曲面を描画
    # ax.plot_surface(X, Y, Z, cmap = "summer")
    # # 底面に等高線を描画
    # ax.contour(X, Y, Z, colors = "black", offset = -1)
    plt.savefig(file_name)
    plt.close()


if __name__ == "__main__":
    df = pd.read_csv(sys.argv[1], sep=',', skipinitialspace=True)
    print(df.keys())
    print(df)
    print(f"acc:                        {set(df['acc'])}")
    print(f"speed:                      {set(df['speed'])}")
    # print(f"dim:                        {set(df['dim'])}")
    print(f"rotate_update_interval:     {set(df['rotate_update_interval'])}")
    print(f"point:                      {set(df['point'])}")
    # print(f"d_dt:                       {set(df['d_dt'])}")
    print(f"angle_init:                 {set(df['angle_init'])}")
    print(f"rotate_angle:               {set(df['rotate_angle'])}")
    print(f"maxdim:                     {set(df['maxdim'])}")

    pd.set_option("display.max_colwidth", 1000)
    print(df.loc[[df['prop_size'].idxmax()]])
    print("\n----- logistic -----")
    xs_keys = ["acc", "speed", "rotate_update_interval", "point", "angle_init", "rotate_angle", "maxdim"]
    print(xs_keys)
    logistic(deepcopy(df), xs_keys, ['prop_size'])

    for acc in set(df['acc']):
        for speed in set(df['speed']):
            for point in set(df['point']):
                for maxdim in set(df['maxdim']):
                    three_d_fig(deepcopy(df.query(f"maxdim == {maxdim}").query(f"acc == {acc}").query(f"speed == {speed}").query(f"point == {point}").query("angle_init == 0")), "rotate_update_interval", "rotate_angle", "prop_size", f"./result/prop_{maxdim}_{acc}_{speed}_{point}.pdf")
                three_d_fig(deepcopy(df.query(f"acc == {acc}").query(f"speed == {speed}").query(f"point == {point}").query("angle_init == 0")), "rotate_update_interval", "rotate_angle", "etsi_size", f"./result/etsi_{acc}_{speed}_{point}.pdf")

    for maxdim in set(df['maxdim']):
        data = list(df.query(f"maxdim == {maxdim}")["time"])
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.hist(data, bins=int(1 + math.log2(len(data))), histtype='barstacked', ec='black')
        plt.savefig(f"./result/time_{maxdim}.pdf")
        plt.close()
