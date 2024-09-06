import airsim
import cv2
import time
import rosbag
import rospy
from sensor_msgs.msg import Imu, NavSatFix
from cv_bridge import CvBridge
import numpy as np
from scipy.spatial.transform import Rotation as R
import sys
import signal

# 初始化AirSim客户端
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# 设置双目相机
camera_name_left = "down_center"
camera_name_right = "down_right"
vehicle_name = "SimpleFlight"

# 创建ROS BAG文件
bag = rosbag.Bag('/home/jingye/Documents/AirSim/rosbags/mountains.bag', 'w')
bridge = CvBridge()

has_printed_calib_info = False
rospy.init_node('airsim_rosbag_recorder', anonymous=True)

# 确保我们按照时间顺序获取不同传感器的数据
start_time = time.time()


def signal_handler(sig, frame):
    print('Stopping recording...')
    bag.close()  # 关闭bag文件以保存索引
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


def get_stereo_images():
    """获取双目相机的图像，并同步曝光"""
    responses = client.simGetImages([
        airsim.ImageRequest(camera_name_left, airsim.ImageType.Scene, False, False),
        airsim.ImageRequest(camera_name_right, airsim.ImageType.Scene, False, False)
    ], vehicle_name)
    if len(responses) == 2:
        left_image_response = responses[0]
        right_image_response = responses[1]
        return left_image_response, right_image_response
    return None, None


def save_image_to_bag(left_image_response, right_image_response, timestamp):
    """将双目相机的图像保存到 ROS bag 中"""
    # try:
    # 转换左目图像
    img_left = np.frombuffer(left_image_response.image_data_uint8, dtype=np.uint8).reshape(left_image_response.height, left_image_response.width, 3)
    img_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
    img_msg_left = bridge.cv2_to_imgmsg(img_left, encoding="mono8")
    img_msg_left.header.stamp = rospy.Time.now()

    # 转换右目图像
    img_right = np.frombuffer(right_image_response.image_data_uint8, dtype=np.uint8).reshape(right_image_response.height, right_image_response.width, 3)
    img_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)
    img_msg_right = bridge.cv2_to_imgmsg(img_right, encoding="mono8")
    img_msg_right.header.stamp = rospy.Time.now()

    cv2.imshow("Left", img_left)
    cv2.imshow("Right", img_right)
    cv2.waitKey(1)

    # 写入 bag
    bag.write('/cam4/image_raw', img_msg_left, t=rospy.Time.from_sec(timestamp))
    bag.write('/cam5/image_raw', img_msg_right, t=rospy.Time.from_sec(timestamp))

    left_cam_pose = client.simGetCameraInfo(camera_name_left, vehicle_name=vehicle_name).pose
    right_cam_pose = client.simGetCameraInfo(camera_name_right, vehicle_name=vehicle_name).pose

    gt = client.simGetGroundTruthKinematics(vehicle_name=vehicle_name)
    imu_pos = gt.position.to_numpy_array()
    imu_ori = gt.orientation.to_numpy_array()
    global has_printed_calib_info
    if not has_printed_calib_info:
        left_cam_pos = left_cam_pose.position.to_numpy_array()
        right_cam_pos = right_cam_pose.position.to_numpy_array()
        left_cam_ori = left_cam_pose.orientation.to_numpy_array()  # xyzw
        right_cam_ori = right_cam_pose.orientation.to_numpy_array()
        left_cam_pose_mat = np.eye(4)
        left_cam_pose_mat[:3, :3] = R.from_quat(left_cam_ori).as_matrix()
        left_cam_pose_mat[:3, 3] = left_cam_pos
        right_cam_pose_mat = np.eye(4)
        right_cam_pose_mat[:3, :3] = R.from_quat(right_cam_ori).as_matrix()
        right_cam_pose_mat[:3, 3] = right_cam_pos
        imu_pose_mat = np.eye(4)
        imu_pose_mat[:3, :3] = R.from_quat(imu_ori).as_matrix()
        imu_pose_mat[:3, 3] = imu_pos
        print(f"Left camera extrinsic:\n{np.linalg.inv(imu_pose_mat) @ left_cam_pose_mat}")
        print(f"Right camera extrinsic:\n{np.linalg.inv(imu_pose_mat) @ right_cam_pose_mat}")
        has_printed_calib_info = True
    # except Exception as e:
    #     print(f"Error saving image to bag: {e}")


def get_imu_data():
    """获取 IMU 数据"""
    imu_data = client.getImuData()
    return imu_data


def save_imu_to_bag(imu_data, timestamp):
    """将 IMU 数据保存到 ROS bag 中"""
    imu_msg = Imu()
    imu_msg.header.stamp = rospy.Time.from_sec(timestamp)
    imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x_val
    imu_msg.linear_acceleration.y = imu_data.linear_acceleration.y_val
    imu_msg.linear_acceleration.z = imu_data.linear_acceleration.z_val
    imu_msg.angular_velocity.x = imu_data.angular_velocity.x_val
    imu_msg.angular_velocity.y = imu_data.angular_velocity.y_val
    imu_msg.angular_velocity.z = imu_data.angular_velocity.z_val
    imu_msg.orientation.w = imu_data.orientation.w_val
    imu_msg.orientation.x = imu_data.orientation.x_val
    imu_msg.orientation.y = imu_data.orientation.y_val
    imu_msg.orientation.z = imu_data.orientation.z_val

    bag.write('/imu0', imu_msg, t=rospy.Time.from_sec(timestamp))


def get_gps_data():
    """获取 GPS 数据"""
    gps_data = client.getGpsData()
    return gps_data


def save_gps_to_bag(gps_data, timestamp):
    """将 GPS 数据保存到 ROS bag 中"""
    gps_msg = NavSatFix()
    gps_msg.header.stamp = rospy.Time.from_sec(timestamp)
    gps_msg.latitude = gps_data.gnss.geo_point.latitude
    gps_msg.longitude = gps_data.gnss.geo_point.longitude
    gps_msg.altitude = gps_data.gnss.geo_point.altitude

    bag.write('/gps', gps_msg, t=rospy.Time.from_sec(timestamp))


# 主循环，按指定频率获取数据
try:
    while not rospy.is_shutdown():
        current_time = time.time()
        elapsed_time = current_time - start_time

        # 获取双目图像 (20Hz)
        if elapsed_time % 0.05 < 0.01:  # 20 Hz
            left_img, right_img = get_stereo_images()
            if left_img and right_img:
                save_image_to_bag(left_img, right_img, current_time)

        # 获取 IMU 数据 (100Hz)
        if elapsed_time % 0.01 < 0.001:  # 100 Hz
            imu_data = get_imu_data()
            save_imu_to_bag(imu_data, current_time)

        # 获取 GPS 数据 (10Hz)
        if elapsed_time % 0.1 < 0.01:  # 10 Hz
            gps_data = get_gps_data()
            save_gps_to_bag(gps_data, current_time)

        # 为了避免高 CPU 占用，稍作休眠
        time.sleep(0.001)
except rospy.ROSInterruptException:
    pass
finally:
    # 关闭 ROS bag
    bag.close()
