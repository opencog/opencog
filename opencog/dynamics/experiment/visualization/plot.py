#!/usr/bin/python
import sys
import csv
import matplotlib.pyplot as plt

#{UUID:[STI,LTI,VLTI,CYCLE]
data = {}
#{UUID:[STRENGTH,CONFIDENCE,CYCLE]
heb_data = {}

def plot_sti(uuids):
   for uuid in uuids:
        sti_val = data[uuid][0]
        cycles = data[uuid][3]
        plt.plot(cycles,sti_val,marker=".",label="UUID:"+str(uuid))
   
   xlabel = "Cycle"
   ylabel = u"\u0394STI"
   plt.xlabel(xlabel)
   plt.ylabel(ylabel)
   plt.legend()
   plt.show()

def plot_lti(uuids):
    for uuid in uuids:
        lti_val = data[uuid][1]
        cycles = data[uuid][3]
        plt.plot(cycles,lti_val,marker=".",label="UUID:"+str(uuid))

    xlabel = "Cycle"
    ylabel = u"\u0394LTI"
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.show()

def plot_vlti(uuids):
    for uuid in uuids:
        sti_val = data[uuid][2]
        cycles = data[uuid][3]
        plt.plot(cycles,sti_val,marker=".",label="UUID:"+str(uuid))

    xlabel = "Cycle"
    ylabel = u"\u0394VLTI"
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.show()

def plot_hebstrength(uuids):
    for uuid in uuids:
        strength_val = heb_data[uuid][0]
        cycles = heb_data[uuid][2]
        [int(i) for i in cycles]
        plt.plot(cycles,strength_val,marker=".",label="UUID:"+str(uuid))

    xlabel = "Cycle"
    ylabel = u"\u0394Strength"
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.show()

def set_data(file_path, uuids):
    f = open(file_path,'r')
    global data
    lines = csv.reader(f);
    for line in lines:
        uuid = line[0]
        if uuid in uuids:
            if uuid not in data.keys():
                data[uuid] = [[],[],[],[]]
                continue
            else:
                data[uuid][0].append(line[1])
                data[uuid][1].append(line[2])
                data[uuid][2].append(line[3])
                data[uuid][3].append(line[4])

def set_heb_data(file_path, uuids):
    f = open(file_path,'r')
    global heb_data
    lines = csv.reader(f);
    for line in lines:
        uuid = line[0]
        if uuid in uuids:
            if uuid not in heb_data.keys():
                heb_data[uuid] = [[0.0],[0.0],[0.0]]
                continue
            else:
                heb_data[uuid][0].append(float(line[1]))
                heb_data[uuid][1].append(float(line[2]))
                heb_data[uuid][2].append(float(line[3]))


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print "ERROR: Options are [FILE_PATH] [sti|lti|vlti|hebst] [comma separated uuids]"
        exit()

    path = sys.argv[1]
    if path == "":
        print "ERROR: Path to time seris data not provided."

    plot = sys.argv[2]
    if plot == "":
        print "ERROR: Need plot type.Supported options are:\n\tsti\tlti\tvlti"

    uuid = []
    uuid_str = sys.argv[3]
    if uuid_str == "" :
        print "ERROR: Need comma separted uuids to plot."
    else:
        uuids = uuid_str.split(',');

    if plot != "hebst":
       set_data(path,uuids)

    if plot == "sti":
        plot_sti(uuids)
    elif plot == "lti":
        plot_lti(uuids)
    elif plot == "vlti":
        plot_vlti(uuids)
    elif plot == "hebst":
        set_heb_data(path,uuids)
        plot_hebstrength(uuids)
    else:
        print "ERROR: Unsupported plot type.Supported types are:\n\tsti\tlti\tvlti"








