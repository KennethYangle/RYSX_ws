import argparse 
import re
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt


def main(args):
    f = open(args.log)
    lines = f.readlines() 
    raw_datas = []
    cnt = 0
    is_first = True
    index = []
    for line in lines:
        if line.startswith(args.variable):
            tmp = [float(a) for a in re.findall(r'-?\d+\.?\d*e?[-+]?\d*', line)]
            if is_first:
                data_len = len(tmp)
            if len(tmp) == data_len:
                raw_datas.append(tmp)
                index.append(cnt)
            is_first = False
        cnt += 1
    datas = [[] for i in range(len(raw_datas[0]))]
    for i in range(len(raw_datas)):
        for j in range(len(raw_datas[0])):
            # print(i,j,raw_datas[i][j])
            datas[j].append(raw_datas[i][j])
    # print(datas)
    
    for i in range(len(datas)):
        plt.figure(i)
        plt.plot(index, datas[i])
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('log', help='log file path')
    parser.add_argument('variable', help='variable to plot')
    args = parser.parse_args()
    print(args)
    main(args)