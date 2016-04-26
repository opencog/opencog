#!/usr/bin/python
import csv
import socket
import os
import time
import sh
import sys

LOCATION = "/home/misgana/OPENCOG/opencog/opencog/attention/experiment/data"
# the first one is supposed to be started manually
NOISE_SIZE = 20
data = {} #size, starting_cycle, cycle


# This implements netcat in python.
#
# If you don't now what netcat is, then you should google it.
# Its important and not complicated.
#
def netcat(hostname, port, content) :
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # If the cogserver is down, the connection will fail.
        try:
            s.connect((hostname, port))
        except socket.error as msg:
            print "Connect failed: ", msgs.close()
            return
        s.sendall(content)
        s.shutdown(socket.SHUT_WR)
        while True:
            data = s.recv(1024)
            if not data or data == "":
                    break
        # print "Received:", repr(data)
        # print "Connection closed."
        s.close()

# This how the noise files names are set based on the size
def z_file_name(size):
    return str(size)+"-smokers-"+str(size*5)+"-friendship-noise.scm"

# The experimental code is loading a file called noise.scm
# so lets copy each generated file to the cannonical noise.scm
# on their turn
# creates the following files Anna.data, Bob.data, Edward.data and Frank.data
def switch_noise_file():
    global NOISE_SIZE
    cur_file = LOCATION+"/"+z_file_name(NOISE_SIZE)
    noise_scm = LOCATION+"/noise.scm"
    print "Switching noise file.\n"
    print "cp "+cur_file + " "+noise_scm +"\n"

    os.system("cp "+cur_file + " "+noise_scm)
    NOISE_SIZE = NOISE_SIZE + 10
    if NOISE_SIZE > 900:
        print "Done with all noise files.\n"
        global data
        for namekey in data:
            print "Saving " + namekey+".data \n"
            with open(namekey+".data", "ab+") as f:
                writer = csv.writer(f)
                print "DEBUG"
                print data[namekey]
                writer.writerows(data[namekey])
        print "Exitng.\n"
        netcat("localhost",17001,"shutdown\n") 
        sys.exit()



def restart_ecan():
    print "Restarting congserver and ECAN-exp.\n"
    #os.system("kill -9 $(pgrep cogserver)")
    netcat("localhost",17001,"shutdown\n") 
    time.sleep(3)
    os.system("rm *.data *.log"); 
    cmd = sh.Command("./opencog/cogserver/server/cogserver")
    cmd(_bg = True)
    time.sleep(3)
    print "Server restarted.\n"
    netcat("localhost",17001,"ecan-load\necan-start\n") 
    print "Ecan restarted.\n"
    time.sleep(3)


def append_to_data():
    with open('smokes-fc-result.data','rw') as loglines:
        for line in loglines:
            if 'INTERESTING:' in line:
               info = line.replace('INTERESTING:','') # will leave Name,cycle
               info = info.split(",") # i.e [Name,cycle]
               info[0] = info[0].strip()
               info[1] = info[1].strip().rstrip("\n")
               curdata = [NOISE_SIZE + NOISE_SIZE*5,info[1]]
               global data
               if info[0] in data.keys() and data[info[0]][0] == curdata[0]: # size should be unique.
                   continue
               elif not info[0] in data.keys():
                   data[info[0]] = [];
                   data[info[0]].append(curdata) # i.e append to {Name => [size,cycle]} size is unique
               else:
                   data[info[0]].append(curdata) # i.e append to {Name => [size,cycle]} size is unique
    pass

if __name__=="__main__":
   print "Stop if found is running. Looking for the interesting atoms in smokes-fc-result.data file\n"
   names = ["Anna", "Bob", "Edward", "Frank"]

   while 1:
       # if all are found exit
        if len(names) == 0:
            print "Found all."
            append_to_data()
            switch_noise_file()
            restart_ecan()

        found = []
        with open('smokes-fc-result.data', 'r') as loglines:
            for line in loglines:
                if 'INTERESTING:' in line:
                    for i, name in enumerate(names):
                        # Delete the found names from list 
                        if name in line:
                            del names[i]
                            break

        # Do the above every 10 secs
        time.sleep(10);

