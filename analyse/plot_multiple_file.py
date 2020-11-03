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
    begin_time = [0 for l in range(nlog)]

    for l in range(nlog):
        f = open(args.log[l])
        lines = f.readlines()
        time = 0
        for line in lines:
            if line.startswith("time:"):
                tmp = [float(a) for a in re.findall(r'-?\d+\.?\d*e?[-+]?\d*', line)]
                time = tmp[0]
                if begin_time[l] == 0: begin_time[l] = time
            if time < tmin or time > tmax:
                continue
            if line.startswith(args.variable+":"):
                tmp = [float(a) for a in re.findall(r'-?\d+\.?\d*e?[-+]?\d*', line)]
                raw_datas[l].append(tmp)
                if args.align:
                    index[l].append(time - begin_time[l])
                else:
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
        fig, ax = plt.subplots()
        if args.grid:
            plt.plot([0,640],[240,240], color='#A5A5A5', linewidth=2)
            plt.plot([320,320],[0,480], color='#A5A5A5', linewidth=2)
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
        if args.grid:
            plt.xticks(np.arange(0,642,80))
            plt.yticks(np.arange(0,481,80))
            plt.grid()
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        fig.savefig("err.png", dpi=1200)
        plt.show()

    if not args.self and not args.plotxy:
        for i in range(len(datas[0])):
            fig = plt.figure(i)
            ax = fig.add_subplot(111)
            for l in range(nlog):
                if args.label is not None:
                    plt.plot(index[l], datas[l][i], label=args.label[l], linewidth=args.linewidth)
                else:
                    plt.plot(index[l], datas[l][i], label="{}".format(os.path.basename(args.log[l]).split(".")[0]), linewidth=args.linewidth)
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
            if args.grid:
                plt.grid()
            ax.spines['top'].set_visible(False)
            ax.spines['right'].set_visible(False)
            fig.savefig("{}.png".format(i), dpi=1200)
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('log', nargs='+', help='log file path')
    parser.add_argument('variable', help='variable to plot')
    parser.add_argument('-s', '--self', action='store_true', help='draw the various components of the variable on a figure')
    parser.add_argument('-p', '--plotxy', action='store_true', help='draw a 2-dimensional graph')
    parser.add_argument('-g', '--grid', action='store_true', help='show grid')
    parser.add_argument('-a', '--align', action='store_true', help='align the time axis')
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