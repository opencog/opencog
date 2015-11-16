#!/usr/bin/python
import sys
import csv
import matplotlib.pyplot as plt

#{UUID:[STI,LTI,VLTI,CYCLE]
data = {}

def plot_sti(uuid_set):
   for uuid in uuid_set:
        sti_val = data[uuid][0]
        cycles = data[uuid][3]
        plt.plot(cycles,sti_val,marker="o",label="UUID:"+str(uuid))
   
   xlabel = "Cycle"
   ylabel = "STI"
   plt.xlabel(xlabel)
   plt.ylabel(ylabel)
   plt.legend()
   plt.show()

def plot_lti(uuid_set):
    for uuid in uuid_set:
        lti_val = data[uuid][1]
        cycles = data[uuid][3]
        plt.plot(cycles,lti_val,marker="o",label="UUID:"+str(uuid))

    xlabel = "Cycle"
    ylabel = "LTI"
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.show()

def plot_vlti(uuid_set):
    for uuid in uuid_set:
        sti_val = data[uuid][2]
        cycles = data[uuid][3]
        plt.plot(cycles,sti_val,marker="o",label="UUID:"+str(uuid))

    xlabel = "Cycle"
    ylabel = "VLTI"
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.show()

def set_data(file_path=""):
    f = open(file_path,'r')
    global data
    lines = csv.reader(f);
    for line in lines:
        uuid = line[0]
        if uuid not in data.keys():
            data[uuid] = [[],[],[],[]]

        data[uuid][0].append(line[1])
        data[uuid][1].append(line[2])
        data[uuid][2].append(line[3])
        data[uuid][3].append(line[4])


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print "ERROR: Options are [FILE_PATH] [sti|lti|vlti] [comma separated uuids]"
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
       uuid = uuid_str.split(',');
    
    set_data(path)
    
    if plot == "sti":
        plot_sti(uuid)
    elif plot == "lti":
        plot_lti(uuid)
    elif plot == "vlti":
        plot_vlti(uuid)
    else:
        print "ERROR: Unsupported plot type.Supported types are:\n\tsti\tlti\tvlti"



        

     


