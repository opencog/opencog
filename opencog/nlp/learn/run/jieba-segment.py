#! /usr/bin/python
#
import time
import sys
import glob
import os
sys.path.append("../")
import jieba
jieba.initialize()
jieba.load_userdict("mandarin.txt")

path = sys.argv[1]
path_out = sys.argv[2]

try:
    os.stat(path_out)
except:
    os.mkdir(path_out)

for filename in glob.iglob(path + '*'):
     print ("processing " + filename)
     content = open(filename, 'rb').read()
     words = " ".join(jieba.cut(content.replace(" ","")))
     log_f = open(path_out + os.path.basename(filename) +".seg","wb")
     log_f.write(words.encode('utf-8'))
     log_f.close()
