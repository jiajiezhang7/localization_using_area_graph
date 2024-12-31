#!/usr/bin/env python
import matplotlib.pyplot as plt 
import numpy as np
import math
import pandas as pd
import os
from mpl_toolkits.mplot3d import Axes3D
from sklearn.tree import DecisionTreeRegressor as dtr # 树算法
from sklearn.model_selection import train_test_split # 拆分数据
from sklearn.metrics import accuracy_score # 模型准确度
from sklearn.tree import plot_tree # 树图
from matplotlib import rcParams # 图大小
from sklearn.cluster import KMeans
# 2023-05-10-20-16-52
# 2023-05-24-20-54-47
path="/home/xiefujing/research/area_graph/ws/frameResult2023-05-10-20-16-52withnoise/"
# path="/home/xiefujing/research/area_graph/ws/frameResult_all_withnoise/"

GT_name="GTliosam2023-05-10-20-16-52.txt"
# GT_name="GTliosamall.txt"

success_xy1=1.0
success_xy=0.5
success_angle=10

def plot_3d(X,Y,Z):
    print("start 3d ploting")
    min_Z = min(Z)
    max_Z = max(Z)
    color = [plt.get_cmap("rainbow", 100)(int(float(i-min_Z)/(max_Z-min_Z)*100)) for i in Z]
    plt.set_cmap(plt.get_cmap("rainbow", 100))
    # 绘制三维散点，各个点颜色使用color列表中的值，形状为"."
    # im = ax.scatter(x, y, z, s=100,c=color,marker='.')
    
    fig = plt.figure(dpi=128,figsize=(8,8))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(X, abs(Y), Z, s=5,c=color, cmap="jet", marker="o", label=name)
    plt.show()
def plot_2d(X,Y,Z,name):
    print("start 2d ploting")
    min_Z = min(Z)
    max_Z = max(Z)
    color = [plt.get_cmap("rainbow", 100)(int(float(i-min_Z)/(max_Z-min_Z)*100)) for i in Z]
    color_array=np.array(color)
    size_array =np.array( [int(float(i-min_Z)/(max_Z-min_Z)*100) for i in Z])
    
    print(X.shape)
    print(Y.shape)
    
    print(size_array.shape)
    plt.set_cmap(plt.get_cmap("rainbow", 100))
    # 绘制三维散点，各个点颜色使用color列表中的值，形状为"."
    # im = ax.scatter(x, y, z, s=100,c=color,marker='.')
    
    fig = plt.figure(dpi=128,figsize=(8,8))
    # ax = fig.add_subplot(111, projection='3d')
    ax = fig.add_subplot(111)
    
    ax_scatter=ax.scatter(X, abs(Y), alpha=0.2, s=6*size_array,c=color, cmap="jet", marker="o", label=name)
    plt.colorbar(ax_scatter)
    imgName=path[0:39]+"frameResultImg/"+name[0:13]+".png"
    plt.savefig(imgName)
    plt.cla()
def plot_real_2d(X,Y,name):
    print("start real 2d ploting")
    # min_Z = min(Z)
    # max_Z = max(Z)
    # color = [plt.get_cmap("rainbow", 100)(int(float(i-min_Z)/(max_Z-min_Z)*100)) for i in Z]
    # color_array=np.array(color)
    # size_array =np.array( [int(float(i-min_Z)/(max_Z-min_Z)*100) for i in Z])
    
    print(X.shape)
    print(Y.shape)
    
    # print(size_array.shape)
    plt.set_cmap(plt.get_cmap("rainbow", 100))
    # 绘制三维散点，各个点颜色使用color列表中的值，形状为"."
    # im = ax.scatter(x, y, z, s=100,c=color,marker='.')
    
    fig = plt.figure(dpi=128,figsize=(8,8))
    # ax = fig.add_subplot(111, projection='3d')
    ax = fig.add_subplot(111)
    
    ax_scatter=ax.scatter(X, abs(Y), alpha=0.5,  cmap="jet", marker="o", label=name)
    # plt.colorbar(ax_scatter)
    imgName=path[0:39]+"frameResultImg/"+name[0:13]+"half.png"
    # plt.savefig(imgName)
    plt.show()
    
    plt.cla()
def ML(allrescueRoom):
    X=allrescueRoom[:  , [6,7,8,9,10,11,12,13,14,15]]
    y=allrescueRoom[: , -1]
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size = 0.2, random_state = 0)
    model = dtr(max_depth=4)
    model.fit(X_train, y_train)
    training_scores=model.score(X_train,y_train)
    testing_scores=model.score(X_test,y_test)
    print(training_scores)
    print(testing_scores)
    rcParams['figure.figsize'] = (16,14)
    plot_tree(model)
    plt.show()

    plt.savefig(path+'tree_visualization.png') 
def only_see_angle():
    # fig = plt.figure(dpi=128,figsize=(8,8))
    fig = plt.figure()
    
    ax0 = fig.add_subplot(111)
    # ax.scatter(X, abs(Y), Z, s=5,c=color, cmap="jet", marker="o", label=name)
    # imgName=path[0:39]+"frameResultImg/"+name[0:13]+"half.png"
    # plt.savefig(imgName)
    # plt.show()
# [number of frame, number of function]
def plot_overall(overallResultxy,overallResultangle):
    overallResultxy=np.array(overallResultxy)
    overallResultangle=np.array(overallResultangle)
    # overallResultxyimu=np.array(overallResultxyimu)
    # overallResultangleimu=np.array(overallResultangleimu)
    print("shape= ",overallResultxy.shape,"shape[0]=",overallResultxy.shape[0])
    
    # print("overallResultxy")
    # print(overallResultxy)
    
    fig = plt.figure()
    axes = fig.subplots(nrows=1, ncols=2)
    for func in range(len(overallResultxy[0])):
        color = plt.cm.Set1(func)
        print("Score function ",func,"------------------------------------")
        p = sum(i <success_xy for i in overallResultxy[:,func])
        # print("len(overallResultxy[0]) =",len(overallResultxy[0]))
        print("xy<"+str(success_xy)+"m percentage=  ",p/overallResultxy.shape[0])
        p = sum(i <success_xy1 for i in overallResultxy[:,func])
        print("xy<"+str(success_xy1)+"m percentage=  ",p/overallResultxy.shape[0])
        
        p = sum(i <success_angle for i in overallResultangle[:,func])
        print("angle<"+str(success_angle)+ "degree percentage=  ",p/overallResultxy.shape[0])
        p = sum((i <success_angle for i in overallResultangle[:,func]) and( j<1.0 for j in overallResultxy[:,func]))
        print("angle<"+str(success_angle)+ "degree and xy<1.0 percentage=  ",p/overallResultxy.shape[0])
        

        
        # p = sum(i <success_xy for i in overallResultxyimu[:,func])
        # print("xy_imu<"+str(success_xy)+"m percentage=  ",p/overallResultxy.shape[0])
        # p = sum(i <success_angle for i in overallResultangleimu[:,func])
        # print("angle_imu< "+str(success_angle)+ "degree percentage=  ",p/overallResultxy.shape[0])
        

        # plt.subplot(1,2, 1)
        x=[n for n in range(overallResultxy.shape[0])]
        
        axes[0].scatter(x,overallResultxy[:,func],label="score function"+str(func+1),c=color)
        axes[0].set_xlabel("LiDAR frame")
        axes[0].set_ylabel("distance error [m]")
        # plt.subplot(1,2, 2)
        axes[1].scatter(x,overallResultangle[:,func],label="score function"+str(func+1),c=color)
        axes[1].set_xlabel("LiDAR frame")
        axes[1].set_ylabel("yaw error [deg]")
    
    # x_=[0 for n in range(len(overallResultxyimu))]
    # plt.subplot(1,2, 1)
    # plt.scatter(x,overallResultxy)
    # plt.xlabel("LiDAR frame")
    # plt.ylabel("distance error [m]")
    # plt.legend()
    # plt.subplot(1,2, 2)
    # plt.scatter(x,overallResultangle)
    # plt.xlabel("LiDAR frame")
    # plt.ylabel("yaw error [deg]")
    lines, labels = fig.axes[-1].get_legend_handles_labels()
    fig.legend(lines,labels,loc='upper right')

    plt.show()    
def k_means(X,gt):
    X=X.reshape(-1,1)
    y_pred = KMeans(n_clusters=2, random_state=0).fit_predict(X)
    y_ = KMeans(n_clusters=2, random_state=0).fit(X)
    
    l = 0
    m = 0
    n = 0
    for i in range(0, len(y_pred)):
        if y_pred[i] == 0:
            l += 1
        elif y_pred[i] == 1:
            m += 1
        elif y_pred[i] == 2:
            n += 1
    print(l, m, n)
    print(y_.cluster_centers_)
    print("difference = ",(y_.cluster_centers_-gt)%180)
    # for k in range(2):
    #     my_members = y_pred == k
    #     print(len(X[my_members]))
    #     print(len([i for i in range(len(X[my_members]))]))
    #     plt.scatter([i for i in range(len(X[my_members]))],X[my_members],  marker='.')
    # plt.show()
    plt.hist((X-gt+360)%180,range(0,180,2))
    plt.show()
def test_one_frame_kmeans():
    filename_=path+"1683721195.25rescueRoom.txt"
    rescueRoom=np.loadtxt(filename_,delimiter=',', dtype=float)#load txt
        
    inside_aver=rescueRoom[:,7]/rescueRoom[:,6]
    outside_aver=rescueRoom[:,9]/rescueRoom[:,8]
    score_sum=rescueRoom[:,7]+rescueRoom[:,9]
    ratio=rescueRoom[:,6]/rescueRoom[:,8]
    total_score=rescueRoom[:,12]/score_sum
    rescueRoom=np.column_stack((rescueRoom, inside_aver,outside_aver,score_sum,ratio,total_score))
    
    rescueRoomSort = rescueRoom[-np.argsort(rescueRoom[:,17])]
    np.set_printoptions(threshold=np.inf)
    GT=np.loadtxt("../../../GT/GTliosam2023-05-24-20-54-47.txt",delimiter='\t')
    where_res=np.where(GT==rescueRoom[0,0])
    gt=[(GT[where_res[0][0],3]+360)%360,GT[where_res[0][0],1],GT[where_res[0][0],2]]
    
    slice_start=int(rescueRoomSort.shape[0]/18)
    # slice_start=100
    rescueRoomSort=rescueRoomSort[0:slice_start]
    print("shape of rescueRoomSort", rescueRoomSort.shape)
    k_means(rescueRoomSort[:,1],gt[0])
    
    rescueRoomSort[:,1:4]=rescueRoomSort[:,1:4]-gt
    errorxy=np.sqrt( rescueRoomSort[:,2]* rescueRoomSort[:,2]+ rescueRoomSort[:,3]* rescueRoomSort[:,3])
    erroryaw= (rescueRoomSort[:,1])%180
    rescueRoomSort[:,1]= (rescueRoomSort[:,1])%180
    rescueRoomSort=np.column_stack((rescueRoomSort,errorxy))
    
    # rescueRoomSort = rescueRoomSort[np.argsort(-rescueRoomSort[:,17])]
    

    
    print(rescueRoomSort.shape)

def get_total_score(rescueRoom,score_sum):
        # total_score0=rescueRoom[:,12]/rescueRoom[:,9]
        # total_score1=rescueRoom[:,12]
        # total_score2=rescueRoom[:,10]*rescueRoom[:,12]
        # total_score3=rescueRoom[:,10]/score_sum/rescueRoom[:,11]
        # total_score4=rescueRoom[:,12]*rescueRoom[:,10]/score_sum/rescueRoom[:,11]
        # total_score5=rescueRoom[:,12]/score_sum/rescueRoom[:,11]
        # total_score6=rescueRoom[:,12]/rescueRoom[:,11]
        # total_score7=rescueRoom[:,10]/rescueRoom[:,11]
        # total_score8=rescueRoom[:,12]/rescueRoom[:,11]
        # total_score9=rescueRoom[:,12]/rescueRoom[:,7]
        # total_score10=rescueRoom[:,12]/score_sum
        # total_score=np.column_stack(( total_score0,total_score1,total_score2,total_score3,total_score4,total_score5,total_score6,total_score7,total_score8,total_score9,total_score10))
        total_score0=1/score_sum
        total_score1=rescueRoom[:,12]
        # total_score2=rescueRoom[:,10]*rescueRoom[:,12]
        # total_score3=rescueRoom[:,10]/score_sum/rescueRoom[:,11]
        # total_score4=rescueRoom[:,12]*rescueRoom[:,10]/score_sum/rescueRoom[:,11]
        # total_score5=rescueRoom[:,12]/score_sum/rescueRoom[:,11]
        # total_score6=rescueRoom[:,12]/rescueRoom[:,11]
        # total_score7=rescueRoom[:,10]/rescueRoom[:,11]
        # total_score8=rescueRoom[:,12]/rescueRoom[:,11]
        # total_score9=rescueRoom[:,12]/rescueRoom[:,7]
        total_score2=rescueRoom[:,12]/score_sum
        total_score3=1/rescueRoom[:,11]
        total_score=np.column_stack(( total_score0,total_score1,total_score2,total_score3))
        # print("total score")
        # print(total_score) 
        return total_score
        
def traverseTotalScoreFunction(rescueRoom):
    result_xy=[]
    result_angle=[]
    result_xy_imu=[]
    result_angle_imu=[]
    for i in range(4):
        rescueRoomSort=rescueRoom[np.argsort(-rescueRoom[:,17+i])]
        result_xy.append(rescueRoomSort[0,-1])
        result_angle.append(rescueRoomSort[0,1])
        where_angle=np.where(rescueRoomSort[:,1]<5)
        
        rescueRoomSort=rescueRoomSort[where_angle]
        rescueRoomSort = rescueRoomSort[np.argsort(-rescueRoomSort[:,17+i])]
        # result_xy_imu.append(rescueRoomSort[0,-1])
        # result_angle_imu.append(rescueRoomSort[0,1])
    # print( result_xy,result_angle,result_xy_imu,result_angle_imu   )
    # return result_xy,result_angle,result_xy_imu,result_angle_imu   
    return result_xy,result_angle   
      
if __name__ == '__main__':
    allFunctionoverallResultxy=[]
    allFunctionoverallResultangle=[]
    # allFunctionoverallResultxyimu=[]
    # allFunctionoverallResultangleimu=[]
    overallResultxy=[]
    overallResultangle=[]
    # overallResultxyimu=[]
    # overallResultangleimu=[]
    # test_one_frame_kmeans()
    
    #timestamp, yaw angle, initial postion xy, rescue result xy
   
    # rescueRoom=np.loadtxt(path+"/1668424054.93rescueRoom.txt",delimiter=',',dtype = float)#load txt
    # GT=np.loadtxt("../../../GT/GTliosam.txt",delimiter='\t')
    # # for i in range(GT)
    # where_res=np.where(GT==rescueRoom[0,0])
    # gt=[GT[where_res[0][0],1],GT[where_res[0][0],2]]
    # resultxy=rescueRoom[:,4:6]
    # errorxy=np.linalg.norm(x=resultxy-gt, ord=2, axis=1, keepdims=False)
    # # s = pd.cut(errorxy, bins=[x for x in range(10 + 1)])
    # bins=[0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,1.5,3,4,5,6,7,8,9,10,10000000]
    # s = pd.cut(errorxy, bins)
    # labels = [str(bins[i]) + '-' + str(bins[i+1]) for i in range(len(bins)-1) ]
    # values = s.value_counts().values
    
    # df = pd.DataFrame(values, index=labels)
    # df.plot(kind='bar', legend=False)
    # plt.xticks(rotation=0)
    # plt.ylabel('numbers')
    # plt.xlabel('interval')
    # plt.show()

    # print(labels)
    # print(s.value_counts())
    # print(errorxy[0])
    # 0  1          2                3              4               5                   6                               7                       8                            9                         10                              
    #ts angle guessx guessy resultx resulty          numofinside insidescore numofoutside outsidescore     inside total range  
    #  11                                       12
    #outside total error           turkey score
    # 13                        14                                     15                                                16 ratio                                         
    # insideaver outsideaver insidescore+outsidescore  numofinside/numofoutside  
    # 17 total score, the more the better                                                                    18
    # ratio/insidescore/outsidescore*inside_total_range                                 errorxy
    
    fileNames=os.listdir(path)
    fileNames.sort(key=lambda x:int(x[:-17]))
    for name in fileNames:
        fileName=path+name
        print(name)
        rescueRoom=np.loadtxt(fileName,delimiter=',', dtype=float)#load txt
        
        inside_aver=rescueRoom[:,7]/rescueRoom[:,6]
        outside_aver=rescueRoom[:,9]/rescueRoom[:,8]
        score_sum=rescueRoom[:,7]+rescueRoom[:,9]
        ratio=rescueRoom[:,6]/rescueRoom[:,8]
        # total_score=ratio*ratio*ratio/inside_aver/outside_aver*np.log(rescueRoom[:,10]*rescueRoom[:,10]*rescueRoom[:,10])
        # total_score=ratio*ratio*ratio*rescueRoom[:,10]*rescueRoom[:,10]/np.exp(inside_aver)/np.exp(outside_aver)
        # total_score=ratio*ratio*ratio/inside_aver/outside_aver*np.log(rescueRoom[:,10]*rescueRoom[:,10]*rescueRoom[:,10])
        total_score=get_total_score(rescueRoom,score_sum)
        # total_score=rescueRoom[:,12]/score_sum

        rescueRoom=np.column_stack((rescueRoom, inside_aver,outside_aver,score_sum,ratio,total_score))
        rescueRoomSort = rescueRoom[np.argsort(-rescueRoom[:,17])]
        
        print(rescueRoomSort[0,1],',',rescueRoomSort[0,2],',',rescueRoomSort[0,3])
        
        np.set_printoptions(threshold=np.inf)
        # GT=np.loadtxt("../../../GT/GTliosam2022-11-14-19-07-34.txt",delimiter='\t')
        GT=np.loadtxt("../../../GT/"+GT_name,delimiter='\t')
        where_res=np.where(GT==rescueRoom[0,0])
        #GT (x,y,angle)
        # gt(angle,x,y)angle[-180,180]
        #  rescueRoomSort[:,1] \in [0,360]
        # gt=[(GT[where_res[0][0],3]+360)%360,GT[where_res[0][0],1],GT[where_res[0][0],2]]
        gt=[GT[where_res[0][0],3],GT[where_res[0][0],1],GT[where_res[0][0],2]]
        
        rescueRoom[:,2:4]=abs(rescueRoom[:,2:4]-gt[1:3])
        rescueRoom[:,1]=rescueRoom[:,1]
        if gt[0]<0:
            gt[0]=360+gt[0]
        for i in range(len(rescueRoom[:,1])):
            if abs(rescueRoom[i,1]-gt[0])>180.0:
                rescueRoom[i,1]=360-abs(rescueRoom[i,1]-gt[0])
            else:
                rescueRoom[i,1]=abs(rescueRoom[i,1]-gt[0])
        errorxy=np.sqrt( rescueRoom[:,2]* rescueRoom[:,2]+ rescueRoom[:,3]* rescueRoom[:,3])
        erroryaw= rescueRoom[:,1]
        rescueRoom=np.column_stack((rescueRoom,errorxy))
        # result_xy,result_angle,result_xy_imu,result_angle_imu=traverseTotalScoreFunction(rescueRoom)
        result_xy,result_angle=traverseTotalScoreFunction(rescueRoom)
        
        # rescueRoomSort = rescueRoomSort[np.argsort(-rescueRoomSort[:,17])]
        overallResultxy.append(result_xy)
        overallResultangle.append(result_angle)
        
        # where_angle=np.where(rescueRoomSort[:,1]<10)
        # print(where_angle)
        # print("size of rescueRoom before whereis = %d",rescueRoomSort.size)
        
        # rescueRoomSort=rescueRoomSort[where_angle]
        # rescueRoomSort = rescueRoomSort[np.argsort(rescueRoomSort[:,1])]
        # rescueRoomSort=rescueRoomSort[0:int(rescueRoomSort.shape[0]/10)]
        # print("size of rescueRoom after whereis = %d",rescueRoomSort.size)
        # rescueRoomSort = rescueRoomSort[np.argsort(-rescueRoomSort[:,17])]
        # overallResultxyimu.append(result_xy_imu)
        # overallResultangleimu.append(result_angle_imu)
        slice_start=int(errorxy.shape[0]/20)
        rescueRoomSort=rescueRoomSort[0:slice_start]
        
        
        # plot_3d(errorxy[0:slice_start],erroryaw[0:slice_start],rescueRoomSort[0:slice_start,15])
        # plot_2d(rescueRoomSort[:,16][0:slice_start],rescueRoomSort[:,1][0:slice_start],rescueRoomSort[:,10][0:slice_start]/rescueRoomSort[:,13][0:slice_start],name)
       
        # plot_2d(rescueRoomSort[:,18][0:slice_start],rescueRoomSort[:,1][0:slice_start],rescueRoomSort[:,17][0:slice_start],name)
        
        # plot_3d(rescueRoomSort[:,16][0:slice_start],rescueRoomSort[:,1][0:slice_start],rescueRoomSort[:,6][0:slice_start])
        # plot_real_2d(rescueRoomSort[:,16][0:slice_start],rescueRoomSort[:,10][0:slice_start],name)
        # ML(rescueRoomSort)
        
        
        
        # Z = np.meshgrid(rescueRoomSort[:,15],rescueRoomSort[:,15])
        # Z=np.array(Z)
        # print(Z.shape)

        #作图
        # score_max=np.max(Z)
        # Z=Z/score_max
        # plt_figure=plt.scatter(errorxy,erroryaw,rescueRoomSort[:,15],alpha=1,cmap="rainbow")     #生成表面， alpha 用于控制透明度
        # plt.colorbar(plt_figure)


        #设定显示范围
        # plt.set_xlabel('X')
        # ax4.set_xlim(-6, 4)  #拉开坐标轴范围显示投影
        # plt.set_ylabel('Y')
        # ax4.set_ylim(-4, 6)
        # plt.set_zlabel('Z')
        # ax4.set_zlim(-3, 3)

        # plt.show()

        # np.insert(rescueRoomSort, 8, values=errorxy, axis=1)
        # rescueRoomSort[:,8]=rescueRoomSort[:,8]/rescueRoomSort[:,6]
        # rescueRoomSort = rescueRoomSort[np.argsort(rescueRoomSort[:,6])]
        
        # print(rescueRoomSort[0:10,16])
        # imgName=path[0:39]+"frameResultImg/"+name[0:13]+".png"
        # plt.ylabel('numbers')
        # plt.xlim(0,55)
        # plt.scatter(rescueRoomSort[:,16],erroryaw,rescueRoomSort[:,15],s=15)
        # print(rescueRoomSort[:,16])
        # plt.savefig(imgName)
        # plt.cla()
        # plt.axis('equal')
        # plt.xlim(0,55)
        # imgName=path[0:39]+"frameResultImg/"+name[0:13]+"half.png"
        # plt.scatter(rescueRoomSort[0:1000,16],rescueRoomSort[0:1000, 6],s=15)
        # plt.savefig(imgName)
        # plt.cla()
        # plt.scatter(rescueRoomSort[:,16],rescueRoomSort[:,10]*rescueRoomSort[:,10],s=0.03)
        # plt.show()
        
        # plt.figure()
        # grid = plt.GridSpec(1,2)
        # plt.subplot(grid[0, 0])
        # plt.gca().set_aspect('equal')
        # plt.scatter(rescueRoomSort[:,16],rescueRoomSort[:,6],s=0.003)
        # plt.subplot(grid[0, 1])
        # plt.gca().set_aspect('equal')
        # plt.scatter(rescueRoomSort[0:1000,16],rescueRoomSort[0:1000,6],s=0.003)
        # plt.subplot(grid[1,1])
        # plt.plot(pose_time,pose_delta_theta)
        # plt.savefig(imgName)
        # plt.cla()


        # rescueRoomSort[:,8]=rescueRoomSort[:,8]/rescueRoomSort[:,6]
        # imgName=path[0:39]+"frameResultImg/"+name[0:13]+"ratio.png"
        # plt.scatter(rescueRoomSort[:,7],rescueRoomSort[:, 6 ],s=2)
        # plt.savefig(imgName)
        # plt.cla()
        # s = pd.cut(errorxy, bins=[x for x in range(10 + 1)])
        # bins=[0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,1.5,3,4,5,6,7,8,9,10,20,30,40,50,60,70,80,10000]
        # print(rescueRoomSort[0])
        
        # s = pd.cut(errorxy, bins)
        # labels = [str(bins[i]) + '-' + str(bins[i+1]) for i in range(len(bins)-1) ]
        # values = s.value_counts().values
        # df = pd.DataFrame(values, index=labels)
        # df.plot(kind='bar', legend=False)
        # plt.xticks(fontsize=6,rotation=0)
        # plt.ylabel('numbers')
        # plt.xlabel('error interval')
        # plt.savefig(imgName)
        # # print(errorxy[0])
        
        # # s = pd.cut(errorxy[0:int(int(errorxy.shape[0])/3) ], bins)
        # s = pd.cut(errorxy[0:20 ], bins)
        
        # labels = [str(bins[i]) + '-' + str(bins[i+1]) for i in range(len(bins)-1) ]
        # values = s.value_counts().values
        # df = pd.DataFrame(values, index=labels)
        # df.plot(kind='bar', legend=False)
        # plt.xticks(fontsize=6,rotation=0)
        # plt.ylabel('numbers')
        # plt.xlabel('error interval')
        # imgName=path[0:39]+"frameResultImg/"+name[0:13]+"half.png"
        # plt.savefig(imgName)
        
        
        

        # plt.show()
    # plot_overall(overallResultxy,overallResultangle,overallResultxyimu,overallResultangleimu)
    plot_overall(overallResultxy,overallResultangle)
    