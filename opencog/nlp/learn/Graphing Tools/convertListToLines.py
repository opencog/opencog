#When lists are output from scheme to a text file, they do not contain newlines.
#This file adds newlines after each object in the list.
fp=open("word-frequencies.txt","r")
a=fp.readline()
fw=open("word-frequencies2.txt","w+")
fw.write(a.replace(' ','\n'))