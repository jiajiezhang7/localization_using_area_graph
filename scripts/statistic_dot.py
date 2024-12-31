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
path="/home/xiefujing/research/area_graph/ws/frameResult2023-05-10-20-16-52withnoise/1683721028.39rescueRoom.txt"
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
    plt.xlabel("Distance Error[m]")
    plt.ylabel("Angle Error[]")
    ax = plt.gca()
    ax.set_aspect(0.02)
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


def get_total_score(rescueRoom,score_sum):
        total_score0=1/score_sum
        total_score1=rescueRoom[:,12]

        total_score2=rescueRoom[:,12]/score_sum
        total_score3=1/rescueRoom[:,11]
        total_score=np.column_stack(( total_score0,total_score1,total_score2,total_score3))

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

    overallResultxy=[]
    overallResultangle=[]
    
    # df = pd.DataFrame(values, index=labels)
    # df.plot(kind='bar', legend=False)
    # plt.xticks(rotation=0)
    # plt.ylabel('numbers')
    # plt.xlabel('interval')
    # plt.show()

    # 0  1          2                3              4               5                   6                               7                       8                            9                         10                              
    #ts angle guessx guessy resultx resulty          numofinside insidescore numofoutside outsidescore     inside total range  
    #  11                                       12
    #outside total error           turkey score
    # 13                        14                                     15                                                16 ratio                                         
    # insideaver outsideaver insidescore+outsidescore  numofinside/numofoutside  
    # 17 total score, the more the better                                                                    18
    # ratio/insidescore/outsidescore*inside_total_range                                 errorxy
    
    rescueRoom=np.loadtxt(path,delimiter=',', dtype=float)#load txt
    inside_aver=rescueRoom[:,7]/rescueRoom[:,6]
    outside_aver=rescueRoom[:,9]/rescueRoom[:,8]
    score_sum=1/(rescueRoom[:,7]+rescueRoom[:,9])
    ratio=rescueRoom[:,6]/rescueRoom[:,8]
    total_score=get_total_score(rescueRoom,score_sum)
    rescueRoom=np.column_stack((rescueRoom, inside_aver,outside_aver,score_sum,ratio,total_score))
    
    np.set_printoptions(threshold=np.inf)
    GT=np.loadtxt("../../../GT/"+GT_name,delimiter='\t')
    where_res=np.where(GT==rescueRoom[0,0])
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
    print("blabla")
    print(rescueRoom.shape)
    
    rescueRoomSort = rescueRoom[np.argsort(-rescueRoom[:,15])]
    # slice_start=int(errorxy.shape[0]/20)
    slice_start=int(200)
    
    rescueRoomSort=rescueRoomSort[0:slice_start]
    plot_2d(rescueRoomSort[:,21][0:slice_start],rescueRoomSort[:,1][0:slice_start],rescueRoomSort[:,15][0:slice_start],"lalala")
    
    # for name in fileNames:

    #     total_score=get_total_score(rescueRoom,score_sum)

    #     rescueRoom=np.column_stack((rescueRoom, inside_aver,outside_aver,score_sum,ratio,total_score))
    #     rescueRoomSort = rescueRoom[np.argsort(-rescueRoom[:,17])]
        
    #     print(rescueRoomSort[0,1],',',rescueRoomSort[0,2],',',rescueRoomSort[0,3])
        
    #     np.set_printoptions(threshold=np.inf)
    #     # GT=np.loadtxt("../../../GT/GTliosam2022-11-14-19-07-34.txt",delimiter='\t')
    #     GT=np.loadtxt("../../../GT/"+GT_name,delimiter='\t')
    #     where_res=np.where(GT==rescueRoom[0,0])
    #     #GT (x,y,angle)
    #     # gt(angle,x,y)angle[-180,180]
    #     #  rescueRoomSort[:,1] \in [0,360]
    #     # gt=[(GT[where_res[0][0],3]+360)%360,GT[where_res[0][0],1],GT[where_res[0][0],2]]
    #     gt=[GT[where_res[0][0],3],GT[where_res[0][0],1],GT[where_res[0][0],2]]
        
    #     rescueRoom[:,2:4]=abs(rescueRoom[:,2:4]-gt[1:3])
    #     rescueRoom[:,1]=rescueRoom[:,1]
    #     if gt[0]<0:
    #         gt[0]=360+gt[0]
    #     for i in range(len(rescueRoom[:,1])):
    #         if abs(rescueRoom[i,1]-gt[0])>180.0:
    #             rescueRoom[i,1]=360-abs(rescueRoom[i,1]-gt[0])
    #         else:
    #             rescueRoom[i,1]=abs(rescueRoom[i,1]-gt[0])
    #     errorxy=np.sqrt( rescueRoom[:,2]* rescueRoom[:,2]+ rescueRoom[:,3]* rescueRoom[:,3])
    #     erroryaw= rescueRoom[:,1]
    #     rescueRoom=np.column_stack((rescueRoom,errorxy))
    #     # result_xy,result_angle,result_xy_imu,result_angle_imu=traverseTotalScoreFunction(rescueRoom)
    #     result_xy,result_angle=traverseTotalScoreFunction(rescueRoom)
        
    #     # rescueRoomSort = rescueRoomSort[np.argsort(-rescueRoomSort[:,17])]
    #     # overallResultxy.append(result_xy)
    #     # overallResultangle.append(result_angle)
        
    #     # where_angle=np.where(rescueRoomSort[:,1]<10)
    #     # print(where_angle)
    #     # print("size of rescueRoom before whereis = %d",rescueRoomSort.size)
        
    #     # rescueRoomSort=rescueRoomSort[where_angle]
    #     # rescueRoomSort = rescueRoomSort[np.argsort(rescueRoomSort[:,1])]
    #     # rescueRoomSort=rescueRoomSort[0:int(rescueRoomSort.shape[0]/10)]
    #     # print("size of rescueRoom after whereis = %d",rescueRoomSort.size)
    #     # rescueRoomSort = rescueRoomSort[np.argsort(-rescueRoomSort[:,17])]
    #     # overallResultxyimu.append(result_xy_imu)
    #     # overallResultangleimu.append(result_angle_imu)
    #     slice_start=int(errorxy.shape[0]/20)
    #     rescueRoomSort=rescueRoomSort[0:slice_start]
        
        
    #     # plot_3d(errorxy[0:slice_start],erroryaw[0:slice_start],rescueRoomSort[0:slice_start,15])
    #     # plot_2d(rescueRoomSort[:,16][0:slice_start],rescueRoomSort[:,1][0:slice_start],rescueRoomSort[:,10][0:slice_start]/rescueRoomSort[:,13][0:slice_start],name)
       
    #     # plot_2d(rescueRoomSort[:,18][0:slice_start],rescueRoomSort[:,1][0:slice_start],rescueRoomSort[:,17][0:slice_start],name)
        
    #     # plot_3d(rescueRoomSort[:,16][0:slice_start],rescueRoomSort[:,1][0:slice_start],rescueRoomSort[:,6][0:slice_start])
    #     # plot_real_2d(rescueRoomSort[:,16][0:slice_start],rescueRoomSort[:,10][0:slice_start],name)
    #     # ML(rescueRoomSort)
        
        
        
    #     # Z = np.meshgrid(rescueRoomSort[:,15],rescueRoomSort[:,15])
    #     # Z=np.array(Z)
    #     # print(Z.shape)

    #     #作图
    #     # score_max=np.max(Z)
    #     # Z=Z/score_max
    #     # plt_figure=plt.scatter(errorxy,erroryaw,rescueRoomSort[:,15],alpha=1,cmap="rainbow")     #生成表面， alpha 用于控制透明度
    #     # plt.colorbar(plt_figure)


    #     #设定显示范围
    #     # plt.set_xlabel('X')
    #     # ax4.set_xlim(-6, 4)  #拉开坐标轴范围显示投影
    #     # plt.set_ylabel('Y')
    #     # ax4.set_ylim(-4, 6)
    #     # plt.set_zlabel('Z')
    #     # ax4.set_zlim(-3, 3)

    #     # plt.show()

    #     # np.insert(rescueRoomSort, 8, values=errorxy, axis=1)
    #     # rescueRoomSort[:,8]=rescueRoomSort[:,8]/rescueRoomSort[:,6]
    #     # rescueRoomSort = rescueRoomSort[np.argsort(rescueRoomSort[:,6])]
        
    #     # print(rescueRoomSort[0:10,16])
    #     # imgName=path[0:39]+"frameResultImg/"+name[0:13]+".png"
    #     # plt.ylabel('numbers')
    #     # plt.xlim(0,55)
    #     # plt.scatter(rescueRoomSort[:,16],erroryaw,rescueRoomSort[:,15],s=15)
    #     # print(rescueRoomSort[:,16])
    #     # plt.savefig(imgName)
    #     # plt.cla()
    #     # plt.axis('equal')
    #     # plt.xlim(0,55)
    #     # imgName=path[0:39]+"frameResultImg/"+name[0:13]+"half.png"
    #     # plt.scatter(rescueRoomSort[0:1000,16],rescueRoomSort[0:1000, 6],s=15)
    #     # plt.savefig(imgName)
    #     # plt.cla()
    #     # plt.scatter(rescueRoomSort[:,16],rescueRoomSort[:,10]*rescueRoomSort[:,10],s=0.03)
    #     # plt.show()
        
    #     # plt.figure()
    #     # grid = plt.GridSpec(1,2)
    #     # plt.subplot(grid[0, 0])
    #     # plt.gca().set_aspect('equal')
    #     # plt.scatter(rescueRoomSort[:,16],rescueRoomSort[:,6],s=0.003)
    #     # plt.subplot(grid[0, 1])
    #     # plt.gca().set_aspect('equal')
    #     # plt.scatter(rescueRoomSort[0:1000,16],rescueRoomSort[0:1000,6],s=0.003)
    #     # plt.subplot(grid[1,1])
    #     # plt.plot(pose_time,pose_delta_theta)
    #     # plt.savefig(imgName)
    #     # plt.cla()


    #     # rescueRoomSort[:,8]=rescueRoomSort[:,8]/rescueRoomSort[:,6]
    #     # imgName=path[0:39]+"frameResultImg/"+name[0:13]+"ratio.png"
    #     # plt.scatter(rescueRoomSort[:,7],rescueRoomSort[:, 6 ],s=2)
    #     # plt.savefig(imgName)
    #     # plt.cla()
    #     # s = pd.cut(errorxy, bins=[x for x in range(10 + 1)])
    #     # bins=[0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,1.5,3,4,5,6,7,8,9,10,20,30,40,50,60,70,80,10000]
    #     # print(rescueRoomSort[0])
        
    #     # s = pd.cut(errorxy, bins)
    #     # labels = [str(bins[i]) + '-' + str(bins[i+1]) for i in range(len(bins)-1) ]
    #     # values = s.value_counts().values
    #     # df = pd.DataFrame(values, index=labels)
    #     # df.plot(kind='bar', legend=False)
    #     # plt.xticks(fontsize=6,rotation=0)
    #     # plt.ylabel('numbers')
    #     # plt.xlabel('error interval')
    #     # plt.savefig(imgName)
    #     # # print(errorxy[0])
        
    #     # # s = pd.cut(errorxy[0:int(int(errorxy.shape[0])/3) ], bins)
    #     # s = pd.cut(errorxy[0:20 ], bins)
        
    #     # labels = [str(bins[i]) + '-' + str(bins[i+1]) for i in range(len(bins)-1) ]
    #     # values = s.value_counts().values
    #     # df = pd.DataFrame(values, index=labels)
    #     # df.plot(kind='bar', legend=False)
    #     # plt.xticks(fontsize=6,rotation=0)
    #     # plt.ylabel('numbers')
    #     # plt.xlabel('error interval')
    #     # imgName=path[0:39]+"frameResultImg/"+name[0:13]+"half.png"
    #     # plt.savefig(imgName)
        
        
        

    #     # plt.show()
    # # plot_overall(overallResultxy,overallResultangle,overallResultxyimu,overallResultangleimu)
    # # plot_overall(overallResultxy,overallResultangle)
    