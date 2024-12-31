#!/usr/bin/env python
import matplotlib.pyplot as plt 
import numpy as np
import math
import pandas as pd

if __name__ == '__main__':
    data=np.loadtxt("./0510legoruntime.txt")
    data_start=data[::2]
    data_end=data[1::2]
    print(np.sum((data_end-data_start)/1e6)/data_start.shape[0])