import matplotlib.pyplot as plt
import numpy as np

def loadData(filename):
    inFile=open(filename, 'r')

    y=[]
    for d in inFile:
        y.append(d)
        print(d)
    return y

def plotData(y):
    length = len(y)
    plt.figure(1)
    plt.plot(y,'rx')
    plt.ylim(1300,1350)
    plt.xlim(100, 400)
    plt.xlabel('num')
    plt.ylabel('depth')

    plt.show()


y = loadData('./data.txt')
plotData(y)