import roslib
import rosbag
import rospy
import cv2
import datetime
from datetime import *
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

rosbag_path = '/home/user/Documents/yolov5/data/rosbag/2022-10-21-13-59-30.bag'

image_path = '/home/user/Documents/yolov5/data/rosbag/images/'
image_Time_Stamps_File_path = image_path + 'timestamps.txt'
pointcloud_path = '/home/user/Documents/yolov5/data/rosbag/velodyne_points/'
pointcloud_Time_Stamps_File_path = pointcloud_path + 'timestamps.txt'

image_topic_name = '/hikrobot_camera/image_raw'
pointcloud_topic_name = '/velodyne_points'


def ImageCreator():
    bridge = CvBridge()
    timeStampsFile = open(image_Time_Stamps_File_path, 'w')
    with rosbag.Bag(rosbag_path, 'r') as bag:
        # 要读取的bag文件；
        num = 0
        for topic, msg, t in bag.read_messages():
            if topic == image_topic_name:
                # 图像的topic；
                try:
                    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                except CvBridgeError as e:
                    print(e)

                pointcloud_name = "%010d" % num + ".png"
                cv2.imwrite(image_path + pointcloud_name, cv_image)

                timeStr = "%.9f" % msg.header.stamp.to_sec()
                # %.9f表示小数点后带有9位，可根据精确度需要修改；
                dateTimeStr = int(timeStr[0:10])
                dotTimeStr = timeStr[11:]

                localTime = datetime.fromtimestamp(dateTimeStr)
                localTimeStr = str(localTime)
                timeStamps = localTimeStr + '.' + dotTimeStr

                # 将时间戳写进txt文件内
                timeStampsFile.write(timeStamps)
                timeStampsFile.write('\n')
                num += 1


def PointcloudCreator():
    timeStampsFile = open(pointcloud_Time_Stamps_File_path, 'w')
    with rosbag.Bag(rosbag_path, 'r') as bag:
        # 要读取的bag文件；
        num = 0
        for topic, msg, t in bag.read_messages():
            if topic == pointcloud_topic_name:
                # 点云的topic；
                PointCloud = msg.data
                pointcloud_name = "%010d" % num + ".bin"
                PointCloudFile = open(pointcloud_path + pointcloud_name, 'wb')
                PointCloudFile.write(PointCloud)
                PointCloudFile.close()

                timeStr = "%.9f" % msg.header.stamp.to_sec()
                # %.9f表示小数点后带有9位，可根据精确度需要修改；
                dateTimeStr = int(timeStr[0:10])
                dotTimeStr = timeStr[11:]

                localTimeStr = datetime.fromtimestamp(dateTimeStr)
                localTimeStr = str(localTimeStr)
                timeStamps = localTimeStr + '.' + dotTimeStr

                # 将时间戳写进txt文件内
                timeStampsFile.write(timeStamps)
                timeStampsFile.write('\n')
                num += 1


if __name__ == '__main__':
    image_creator = ImageCreator()
    pointcloud_creator = PointcloudCreator()