import pandas as pd # 数据处理
import numpy as np # 使用数组
import matplotlib.pyplot as plt # 可视化
from matplotlib import rcParams # 图大小
from termcolor import colored as cl # 文本自定义
import os
from sklearn.tree import DecisionTreeRegressor as dtr # 树算法
from sklearn.model_selection import train_test_split # 拆分数据
from sklearn.metrics import accuracy_score # 模型准确度
from sklearn.tree import plot_tree # 树图


rcParams['figure.figsize'] = (25, 20)
path="/home/xiefujing/research/area_graph/ws/frameResult/"
result=np.empty(shape=[0, 12])
def generateData():
    global result
    fileNames=os.listdir(path)
    GT=np.loadtxt("../../../GT/GTliosam.txt",delimiter='\t')
    for name in fileNames:
        fileName=path+name
        rescueRoom=np.loadtxt(fileName,delimiter=',',dtype = float)#load txt
        where_res=np.where(GT==rescueRoom[0,0])
        gt=[GT[where_res[0][0],1],GT[where_res[0][0],2]]
        rescueRoom[:,4:6]=rescueRoom[:,4:6]-gt
        errorxy=np.linalg.norm(x=rescueRoom[:,4:6], ord=2, axis=1, keepdims=False)
        
        rescueRoom=np.column_stack((rescueRoom,errorxy))
        result=np.row_stack((result,rescueRoom))
    
    np.set_printoptions(suppress=True)
    np.set_printoptions(precision=2)   #设精度
    # np.savetxt('data_name', data.reshape(-1, 1), fmt='%.04f')   #保留4位小
    np.savetxt(path+"allframeResult.txt",result ,delimiter=',',fmt='%.02f')
        
    # 0  1          2                3              4               5               6                   7                       8                            9                         10
    #ts angle guessx guessy resultx resulty numofinside insidescore numofoutside outsidescore     inside total range  
    # 11                        12                                      13                                                  14 ratio                                         
    # insideaver outsideaver insidescore+outsidescore  numofinside/numofoutside  
    # 15 total score, the more the better
    # ratio/insidescore/outsidescore*inside_total_range      
def test():
    A=np.zeros((6,6))
    A[:,[1,3]]=6
    print(A)

if __name__ == '__main__':
    # generateData()
    # test()
    allrescueRoom=np.loadtxt(path+"allframeResult.txt",delimiter=',',dtype = float)#load txt
    
    inside_aver=allrescueRoom[:,7]/allrescueRoom[:,6]
    outside_aver=allrescueRoom[:,9]/allrescueRoom[:,8]
    score_sum=allrescueRoom[:,7]+allrescueRoom[:,9]
    ratio=allrescueRoom[:,6]/allrescueRoom[:,8]
    # total_score=ratio*ratio/inside_aver/outside_aver*rescueRoom[:,10]
    allrescueRoom=np.column_stack((allrescueRoom, inside_aver,outside_aver,score_sum,ratio))
    
    X=allrescueRoom[:  , [7,9,10,11,12,13,14]]
    y=allrescueRoom[: , -1]
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size = 0.2, random_state = 0)
    model = dtr(max_depth=4)
    model.fit(X_train, y_train)
    training_scores=model.score(X_train,y_train)
    testing_scores=model.score(X_test,y_test)
    print(training_scores)
    print(testing_scores)
    # plt.subplot(grid[0, 0])
    # plt.gca().set_aspect('equal')
    # plt.scatter(rescueRoomSort[:,16],rescueRoomSort[:,9],s=0.003)
    # plt.subplot(grid[0, 1])
    # plt.gca().set_aspect('equal')
    # plt.scatter(rescueRoomSort[0:1000,16],rescueRoomSort[0:1000, 9],s=0.003)
    # # plt.subplot(grid[1,1])
    # # plt.plot(pose_time,pose_delta_theta)
    # plt.savefig(imgName)
    plot_tree(model)
    # plt.plot(training_scores)
    # plt.plot(testing_scores)
    # plt.show()
    plt.savefig(path+'tree_visualization.png') 