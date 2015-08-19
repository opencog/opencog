#Let's say that you have a list of objects in a file and you need to append 
#their rank to them. This file does exactly that. "fp" denotes the input
#file while "fw" denotes the output file.
fp=open("word-frequencies2.txt","r")
fw=open("word-rank.txt","w+")
i=1
for line in fp:
	string=line
	a=string.strip()
	fw.write(a + " " + `i` + "\n")
	i+=1