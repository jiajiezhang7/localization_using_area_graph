#!/home/xiefujing/anaconda3/envs/py3/bin/python3.6
import rosbag
import rospy
import time
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose

# bagFile="/home/xiefujing/research/area_graph/2022-11-14-19-07-34.bag"
bagFile="/home/xiefujing/research/area_graph/2023-05-10-20-16-52.bag"
# bagFile="/home/xiefujing/research/area_graph/2023-05-24-20-54-47.bag"

times=0
start=1
freq=10#30  send every 3 second
# start_timestamp=1668424118.90 #initialYawAngle:  180#corridor,set to 0 if wants to start from beginning 
# start_timestamp=1668424161.90 #initialYawAngle:  180#corridor,set to 0 if wants to start from beginning 
# start_timestamp=1668424175.00 #initialYawAngle:  180#corridor,set to 0 if wants to start from beginning 

# start_timestamp=1683721209.00 #0510 bag start 
# start_timestamp=1683721035.50 #0510 bag start 

# start_timestamp=1683721166.00 #new bag
# start_timestamp=1683721127.20 #new bag
# start_timestamp=1683721052.20 #new bag
# start_timestamp=1683721076.20 #new bag
# start_timestamp=1683721250.20 #new bag
start_timestamp=1684932891.20
start_timestamp=1684933053.20


# start_timestamp=1668424111.80 #corridor,set to 0 if wants to start from beginning
def callback(msg):
    print("getting done signal")
    global times
    global start
    for topic, msg, t in bag_data:
        # if float(t)>1668424055.93:
        if float(t.to_sec())>start_timestamp:
            if topic == "/hesai/pandar":
                times+=1
                #send every 3 secs
                if times%(freq*start)==0:
                    start=times//freq+1
                    print("sending afters, ts =  %f",float(t.to_sec()))
                    time.sleep(1)
                    pubPC.publish(msg)
                    time.sleep(1)
                    break

if __name__ == '__main__':
    
    rospy.init_node('initDataSender')
# wait for cloudhandler to be ready
    time.sleep(10)
    
    bag = rosbag.Bag(bagFile, "r")
    bag_data = bag.read_messages()    # 利用迭代器返回三个值：{topic标签, msg数据, t时间戳}
    rospy.Subscriber("doneInit", Pose, callback)
    pubPC=rospy.Publisher('/hesai/pandar', PointCloud2, queue_size=1)
    
    # corridor bag time 1668424074.91
    for topic, msg, t in bag_data:
        if float(t.to_sec())>start_timestamp:
        
            if topic == "/hesai/pandar":
                time.sleep(2)
                print("sending first")
                time.sleep(2)
                
                pubPC.publish(msg)
                time.sleep(1)
                break
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()
