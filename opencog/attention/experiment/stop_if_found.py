#!/usr/bin/python
import csv
import socket
import os
import time
import sh
from sh import grep
from sh import mv
import sys

LOCATION = "/home/misgana/OPENCOG/opencog/opencog/attention/experiment/data"
# the first one is supposed to be started manually
noise_size = 10
MAX_NOISE_SIZE = 990
RESULT_STORE_FILE = "raw_result.csv"


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
    global noise_size
    cur_file = LOCATION+"/"+z_file_name(noise_size)
    noise_scm = LOCATION+"/noise.scm"
    print "Switching noise file.\n"
    print "cp "+cur_file + " "+noise_scm +"\n"

    os.system("cp "+cur_file + " "+noise_scm)
    noise_size = noise_size + 10
    if noise_size > MAX_NOISE_SIZE:
        print "Done with all noise files.\n"
        write_separate_result_files()
        print "Exitng.\n"
        sys.exit()

def write_separate_result_files():
    data = {}
    with open(RESULT_STORE_FILE,'r') as r:
        for line in r:
            values = line.split(', ') #size,name,cycle
            person = values[1]  # i.e name
            if person in data.keys():
                data[person].append([values[0],values[2]])
            else:
               data[person] = []
               data[person].append([values[0],values[2]])
    for namekey in data:
            print "Saving " + namekey+".data \n"
            with open(namekey+".data", "ab+") as f:
                writer = csv.writer(f)
                print "DEBUG"
                print data[namekey]
                writer.writerows(data[namekey])



def restart_ecan():
    print "Restarting congserver and ECAN-exp.\n"
    #os.system("kill -9 $(pgrep cogserver)")
    #os.system("rm *.data *.log"); 
    os.system("rm  *.log"); 
    cmd = sh.Command("./opencog/cogserver/server/cogserver")
    cmd(_bg = True)
    time.sleep(3)
    print "Server restarted.\n"
    netcat("localhost",17001,"ecan-load\necan-start\n") 
    print "Ecan restarted.\n"
    time.sleep(3)


def backup_result():
        tmpfile = "result.tmp"
        mv("smokes-fc-result.data",tmpfile)
        interesting = ""
        with open(tmpfile,'r') as loglines:
            for line in loglines:
                if 'INTERESTING:' in line:
                   line = str(noise_size)+", "+line.replace('INTERESTING:','') # will leave Name,cycle
                   interesting  = interesting + line
        with open(RESULT_STORE_FILE,'ab+') as f:
            f.write(interesting)

def get_interesting(name):
    try:
        s=grep("-i","-e","INTERESTING: "+name,"smokes-fc-result.data")
        return True
    except:
        return False

if __name__=="__main__":
   print "Stop if found is running. Looking for the interesting atoms in smokes-fc-result.data file\n"
   names = ["Anna", "Bob", "Edward", "Frank"]
   counter=0
   cont=False
   timeOut=0
   timesTimeOut=0
   switch_noise_file()
   restart_ecan()
   while 1:
        cont=False
        # if all are found exit
        for nm in names:
            if not get_interesting(nm):
                time.sleep(1)
                timeOut=timeOut+1
                cont=True
                break

        if cont and (timeOut<10*60):
            continue

    
        netcat("localhost",17001,"shutdown\n") 
        time.sleep(4)
        if timeOut>=10*60:
            timeOut=0
            timesTimeOut=timesTimeOut+1
            print "TIME OUT...\n"
            print "time out .."+str(timesTimeOut)+"\n"
            print "file count="+str(counter)+"\n"
            switch_noise_file()
            restart_ecan()
            continue

        print "Found all."
        counter=counter+1
        print "file count="+str(counter)+"\n"
        print "time out .."+str(timesTimeOut)+"\n"
        backup_result()
        switch_noise_file()
        restart_ecan()
        timeOut=0
