#!/usr/bin/env python
import  numpy as np
if __name__ == '__main__':
# 0510 bag
    # start_time=1683721047.00
    # end_time=1683721246.00
    #  0524 bag
    start_time=1684932920.00
    # end_time=1684933020.00
    end_time=1694932920.00
    fileLio="/home/xiefujing/research/area_graph/ws/robotPoseResult/LiosamPoseTum.txt"
    fileRobot="/home/xiefujing/research/area_graph/ws/robotPoseResult/robotPoseTum.txt"
    LioResult=np.loadtxt(fileLio,delimiter=' ', dtype=np.float64)#load txt
    RobotResult=np.loadtxt(fileRobot,delimiter=' ', dtype=np.float64)#load txt
    
    whereLio=np.where(start_time<LioResult[:,0])
    whereRobot=np.where(start_time<RobotResult[:,0])
    LioResultTranscated=LioResult[whereLio]
    RobotResultTranscated=RobotResult[whereRobot]
    
    whereLio=np.where(LioResultTranscated[:,0]<end_time)
    whereRobot=np.where(RobotResultTranscated[:,0]<end_time)
    LioResultTranscated=LioResultTranscated[whereLio]
    RobotResultTranscated=RobotResultTranscated[whereRobot]
    np.savetxt("/home/xiefujing/research/area_graph/ws/robotPoseResult/LiosamPoseTumTranscated.txt",LioResultTranscated,fmt='%lf',delimiter=' ')
    np.savetxt("/home/xiefujing/research/area_graph/ws/robotPoseResult/robotPoseTumTranscated.txt",RobotResultTranscated,fmt='%lf',delimiter=' ')