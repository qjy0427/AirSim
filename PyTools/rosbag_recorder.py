import airsim
import cv2
import time
import rosbag
import rospy
from sensor_msgs.msg import Image, Imu, NavSatFix
from cv_bridge import CvBridge

# 初始化AirSim客户端
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# 设置双目相机
camera_name_left = "0"
camera_name_right = "1"
vehicle_name = "SimpleFlight"

# 创建ROS BAG文件
bag = rosbag.Bag('/home/jingye/Documents/AirSim/rosbags/mountains.bag', 'w')
bridge = CvBridge()

has_printed_calib_info = False
left_cam_pose = None
right_cam_pose = None
imu_pos = None
imu_ori = None
gps_pos = None
gt_geo = None
rospy.init_node('airsim_rosbag_recorder', anonymous=True)

try:
    start_time = time.time()
    while True:
        # 获取左相机图像
        response_left = client.simGetImage(camera_name_left, airsim.ImageType.Scene, vehicle_name=vehicle_name)
        left_cam_pose = client.simGetCameraInfo(camera_name_left, vehicle_name=vehicle_name).pose
        if response_left:
            img_left = cv2.imdecode(airsim.string_to_uint8_array(response_left), cv2.IMREAD_COLOR)
            img_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
            img_msg_left = bridge.cv2_to_imgmsg(img_left, encoding="mono8")
            img_msg_left.header.stamp = rospy.Time.now()
            bag.write('/cam4/image_raw', img_msg_left)
            if not has_printed_calib_info and left_cam_pose is not None and right_cam_pose is not None\
                and imu_pos is not None and imu_ori is not None and gps_pos is not None:
                print(f"Left camera image shape: {img_left.shape}")
                print(f"Left camera pose: {left_cam_pose}")
                print(f"Right camera pose: {right_cam_pose}")
                print(f"IMU position: {imu_pos}")
                print(f"IMU orientation: {imu_ori}")
                print(f"GNSS LLH: {gps_pos}")
                print(f"IMU LLH: {gt_geo}")
                left_cam_pos = left_cam_pose.position.to_numpy_array()
                right_cam_pos = right_cam_pose.position.to_numpy_array()
                print(f"Left camera extrinsic position: {left_cam_pos - imu_pos}")
                print(f"Right camera extrinsic position: {right_cam_pos - imu_pos}")
                has_printed_calib_info = True

        # 获取右相机图像
        response_right = client.simGetImage(camera_name_right, airsim.ImageType.Scene, vehicle_name=vehicle_name)
        right_cam_pose = client.simGetCameraInfo(camera_name_right, vehicle_name=vehicle_name).pose
        if response_right:
            img_right = cv2.imdecode(airsim.string_to_uint8_array(response_right), cv2.IMREAD_COLOR)
            img_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)
            img_msg_right = bridge.cv2_to_imgmsg(img_right, encoding="mono8")
            # Convert to grayscale
            img_msg_right.header.stamp = rospy.Time.now()
            bag.write('/cam5/image_raw', img_msg_right)

        # 获取IMU数据
        imu_data = client.getImuData(vehicle_name=vehicle_name)
        gt = client.simGetGroundTruthKinematics(vehicle_name=vehicle_name)
        imu_pos = gt.position.to_numpy_array()
        imu_ori = gt.orientation.to_numpy_array()
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x_val
        imu_msg.linear_acceleration.y = imu_data.linear_acceleration.y_val
        imu_msg.linear_acceleration.z = imu_data.linear_acceleration.z_val
        imu_msg.angular_velocity.x = imu_data.angular_velocity.x_val
        imu_msg.angular_velocity.y = imu_data.angular_velocity.y_val
        imu_msg.angular_velocity.z = imu_data.angular_velocity.z_val
        bag.write('/imu0', imu_msg)

        # 获取GNSS数据
        gnss_data = client.getGpsData(vehicle_name=vehicle_name)
        gps_pos = gnss_data.gnss.geo_point
        gt_geo = client.simGetGroundTruthEnvironment(vehicle_name=vehicle_name).geo_point

        gnss_msg = NavSatFix()
        gnss_msg.header.stamp = rospy.Time.now()
        gnss_msg.latitude = gnss_data.gnss.geo_point.latitude
        gnss_msg.longitude = gnss_data.gnss.geo_point.longitude
        gnss_msg.altitude = gnss_data.gnss.geo_point.altitude
        bag.write('/gps', gnss_msg)

        # 按下 'q' 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # 释放资源
    bag.close()
    client.armDisarm(False)
    client.enableApiControl(False)
    cv2.destroyAllWindows()
