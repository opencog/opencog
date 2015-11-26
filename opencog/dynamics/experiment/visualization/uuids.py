#!/usr/bin/python
import sys
import csv

if __name__ == "__main__":
	file_path = sys.argv[1];
	f = open(file_path,'r')
        uuids = []
	uuids_str = ""
        lines = csv.reader(f);
        for line in lines:
		uuid = line[0]
		if uuid not in uuids:
			uuids.append(uuid)
			uuids_str += uuid+","
	uuids_str = uuids_str[:len(uuids_str)-1]

	print uuids_str



 
