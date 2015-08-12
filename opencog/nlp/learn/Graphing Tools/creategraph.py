#This file draws the actual graphs.
#it needs matplotlib as well as numpy to run.
import numpy
from matplotlib import pyplot as plt
import math
fp=open("tmep.txt","r")
a=[]
b=[]

for line in fp:
	string=line.strip().split()
	a.append(math.log(float(string[0])))
	b.append(math.log(float(string[1])))

a=numpy.asarray(a,dtype=float) #Converts Y-axis variable to numpy array
b=numpy.asarray(b,dtype=float) #Converts X-axis variable to numpy array
coefficients=numpy.polyfit(b,a,1) #This is regression. This function tries to find the best fit line.
polynomial=numpy.poly1d(coefficients) #A 1d polynomial from the above coefficients is created.
#polynomial[0]=14.10
#polynomial[1]=-0.81
ys=polynomial(b) #The equation is put in the form of y=mx+c
plt.plot(b,a,'ro') #The line is actually plotted
plt.plot(b,ys,linewidth=2)
polynomial[1]=-1 #This line and the one below it are used to modify the coefficients of the line equation.
polynomial[0]=11.6 #This allows us to tweak the line better to visually fit the data better.
ys=polynomial(b)
print polynomial
plt.plot(b,ys,linewidth=2)
plt.xlabel("Log (Rank of word when sorted by frequency)") #The X-axis label
plt.ylabel("Log (Frequency of Nth word in wordpairs, when words are sorted by frequency)") #The Y-axis label
plt.title("Words are sorted according to their count.\n The Nth word is taken and the number of times that word occurs in wordpairs is plotted")
#The above line changes the title of the graph
plt.text(8,8,"slope=-0.8879",style='italic',bbox={'facecolor':'blue','alpha':0.5,'pad':10})
plt.text(4,4,"slope=-1",style='italic',bbox={'facecolor':'green','alpha':0.5,'pad':10})
plt.show()