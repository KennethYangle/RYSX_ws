import argparse 
import re
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt


def main(args):
    tmin = 0
    tmax = 1e10
    if args.time is not None:
        trange = [float(a) for a in re.findall(r'-?\d+\.?\d*e?[-+]?\d*', args.time)]
        tmin = trange[0]
        if len(trange) == 2:
            tmax = trange[1]

    f = open(args.log)
    lines = f.readlines() 
    nvar = len(args.variable)
    raw_datas = [[] for v in range(nvar)]
    cnt = 0
    time = 0
    index = [[] for v in range(nvar)]
    data_len = [0 for v in range(nvar)]
    for line in lines:
        for v in range(nvar):
            if line.startswith("time:"):
                tmp = [float(a) for a in re.findall(r'-?\d+\.?\d*e?[-+]?\d*', line)]
                time = tmp[0]
            if time < tmin or time > tmax:
                continue
            if line.startswith(args.variable[v]+":"):
                tmp = [float(a) for a in re.findall(r'-?\d+\.?\d*e?[-+]?\d*', line)]
                # print(tmp)
                if data_len[v]==0:
                    data_len[v] = len(tmp)
                if data_len[v]!=0 and len(tmp) == data_len[v]:
                    raw_datas[v].append(tmp)
                    index[v].append(time)
        cnt += 1
    print(data_len)
    # print(raw_datas)
    datas = [[[] for i in range(len(raw_datas[v][0]))] for v in range(nvar)]
    for v in range(nvar):
        for i in range(len(raw_datas[v])):
            for j in range(len(raw_datas[v][0])):
                # print(i,j,raw_datas[i][j])
                datas[v][j].append(raw_datas[v][i][j])
    # print(datas)
    
    if args.self:
        for v in range(nvar):
            plt.figure(v)
            for i in range(len(datas[v])):
                plt.plot(index[v], datas[v][i], label="{}[{}]".format(args.variable[v], i), linewidth=args.linewidth)
            plt.legend()
        plt.show()

    if args.plotxy:
        for v in range(nvar):
            plt.figure(v)
            plt.plot(datas[v][0], datas[v][1], label="{}[x:y]".format(args.variable[v]), linewidth=args.linewidth)
            if args.range is not None:
                arange = [float(a) for a in re.findall(r'-?\d+\.?\d*e?[-+]?\d*', args.range)]
                plt.axis(arange)
            plt.legend()
        plt.show()

    if not args.self and not args.plotxy:
        for i in range(len(datas[0])):
            plt.figure(i)
            for v in range(nvar):
                if i < len(datas[v]):
                    plt.plot(index[v], datas[v][i], label="{}[{}]".format(args.variable[v], i), linewidth=args.linewidth)
                else:
                    plt.plot(index[v], datas[v][0], label="{}[{}]".format(args.variable[v], 0), linewidth=args.linewidth)
            plt.legend()
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('log', help='log file path')
    parser.add_argument('variable', nargs='+', help='variable to plot')
    parser.add_argument('-s', '--self', action='store_true', help='draw the various components of the variable on a figure')
    parser.add_argument('-p', '--plotxy', action='store_true', help='draw a 2-dimensional graph')
    parser.add_argument('-l', '--linewidth', default=2, type=float, help='line width')
    parser.add_argument('-r', '--range', default=None, help='axises range, work with plotxy. usage: "xmin xmax ymin ymax"')
    parser.add_argument('-t', '--time', default=None, help='time range. usage: "tmin tmax" or tmin')
    args = parser.parse_args()
    print(args)
    main(args)