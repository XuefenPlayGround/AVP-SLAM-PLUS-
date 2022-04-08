#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt


if __name__=="__main__":

    GTList = np.load('GTList.npy')
    poseList = np.load('poseList.npy')

    GTList = np.array(GTList)
    poseList = np.array(poseList)
    print(GTList.shape,poseList.shape)

    plt.subplot(111)

    ax1 = plt.subplot(1, 1, 1)
    ax1.plot(GTList[:,0],GTList[:,1],label='GT')
    ax1.plot(poseList[:,0],poseList[:,1],label='pose')
    ax1.set(xlabel='x')
    ax1.set(ylabel='y')
    ax1.legend()
    
    plt.show()



