import argparse 
import re
import os.path
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

    nlog = len(args.log)
    raw_datas = [[] for l in range(nlog)]
    index = [[] for l in range(nlog)]

    for l in range(nlog):
        f = open(args.log[l])
        lines = f.readlines()
        time = 0
        for line in lines:
            if line.startswith("time:"):
                tmp = [float(a) for a in re.findall(r'-?\d+\.?\d*e?[-+]?\d*', line)]
                time = tmp[0]
            if time < tmin or time > tmax:
                continue
            if line.startswith(args.variable+":"):
                tmp = [float(a) for a in re.findall(r'-?\d+\.?\d*e?[-+]?\d*', line)]
                raw_datas[l].append(tmp)
                index[l].append(time)
    # print(raw_datas)

    datas = [[[] for i in range(len(raw_datas[l][0]))] for l in range(nlog)]
    for l in range(nlog):
        for i in range(len(raw_datas[l])):
            for j in range(len(raw_datas[l][i])):
                # print(i,j,raw_datas[i][j])
                datas[l][j].append(raw_datas[l][i][j])
    # print(datas)

    if args.plotxy:
        for l in range(nlog):
            if args.label is not None:
                plt.plot(datas[l][0], datas[l][1], label=args.label[l], linewidth=args.linewidth)
            else:
                plt.plot(datas[l][0], datas[l][1], label="{}".format(os.path.basename(args.log[l]).split(".")[0]), linewidth=args.linewidth)
        if args.range is not None:
            arange = [float(a) for a in re.findall(r'-?\d+\.?\d*e?[-+]?\d*', args.range)]
            plt.axis(arange)
        if args.title is not None:
            plt.title(args.title)
        if args.xlabel is not None:
            plt.xlabel(args.xlabel)
        if args.ylabel is not None:
            plt.ylabel(args.ylabel)
        plt.legend()
        plt.show()

    if not args.self and not args.plotxy:
        for i in range(len(datas[0])):
            plt.figure(i)
            for l in range(nlog):
                if args.label is not None:
                    plt.plot(index[l], datas[l][i], label=args.label[l], linewidth=args.linewidth)
                else:
                    plt.plot(index[l], datas[l][i], label="{}".format(os.path.basename(args.log[l]).split(".")[0]), linewidth=args.linewidth)
            if args.title is not None:
                plt.title(args.title)
            if args.xlabel is not None:
                plt.xlabel(args.xlabel)
            if args.ylabel is not None:
                plt.ylabel(args.ylabel)
            plt.legend()
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('log', nargs='+', help='log file path')
    parser.add_argument('variable', help='variable to plot')
    parser.add_argument('-s', '--self', action='store_true', help='draw the various components of the variable on a figure')
    parser.add_argument('-p', '--plotxy', action='store_true', help='draw a 2-dimensional graph')
    parser.add_argument('-l', '--linewidth', default=2, type=float, help='line width')
    parser.add_argument('-r', '--range', default=None, help='axises range, work with plotxy. usage: "xmin xmax ymin ymax"')
    parser.add_argument('-t', '--time', default=None, help='time range. usage: "tmin tmax" or tmin')
    parser.add_argument('--label', nargs='+', default=None, help='curve labels')
    parser.add_argument('--title', default=None, help='figure title')
    parser.add_argument('--xlabel', default=None, help='x label')
    parser.add_argument('--ylabel', default=None, help='y label')
    args = parser.parse_args()
    print(args)
    main(args)