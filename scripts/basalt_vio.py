import time
import depthai as dai
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros

rclpy.init()
ros_node = rclpy.create_node('basalt_bridge')
odom_pub = ros_node.create_publisher(Odometry, '/odom', 10)
tf_broadcaster = tf2_ros.TransformBroadcaster(ros_node)

# Create pipeline
with dai.Pipeline() as p:
    fps = 60
    width = 640
    height = 400

    print(f"[basalt_bridge] fps={fps}  resolution={width}x{height}  imu_rate=200 Hz  topic=/odom", flush=True)

    # Define sources and outputs
    left = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=fps)
    right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=fps)
    imu = p.create(dai.node.IMU)
    odom = p.create(dai.node.BasaltVIO)

    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    # Linking
    left.requestOutput((width, height)).link(odom.left)
    right.requestOutput((width, height)).link(odom.right)
    imu.out.link(odom.imu)

    odomQ = odom.transform.createOutputQueue(maxSize=8, blocking=False)

    p.start()
    while p.isRunning():
        transform = odomQ.tryGet()
        if transform is not None:
            pos = transform.getTranslation()
            quat = transform.getQuaternion()
            # print(f"pos: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f} | "
            #       f"quat: qx={quat.qx:.3f}, qy={quat.qy:.3f}, qz={quat.qz:.3f}, qw={quat.qw:.3f}")

            # VIO → ROS frame correction (observed: phys +X→VIO -X, phys -Y→VIO +Z)
            # Rotation matrix: ros_x=-vio_x, ros_y=-vio_z, ros_z=-vio_y
            # Quaternion vector part transforms identically to the position vector.
            rx  = -float(pos.x)
            ry  = -float(pos.z)
            rz  = -float(pos.y)
            rqx =  float(quat.qx)
            rqy =  float(quat.qz)
            rqz =  float(quat.qy)
            rqw =  float(quat.qw)
            print(f"ros: x={rx:.3f}, y={ry:.3f}, z={rz:.3f} | "
                  f"quat: qx={rqx:.3f}, qy={rqy:.3f}, qz={rqz:.3f}, qw={rqw:.3f}")

            stamp = ros_node.get_clock().now().to_msg()

            msg = Odometry()
            msg.header.stamp    = stamp
            msg.header.frame_id = 'odom'
            msg.child_frame_id  = 'base_link'
            msg.pose.pose.position.x    = rx
            msg.pose.pose.position.y    = ry
            msg.pose.pose.position.z    = rz
            msg.pose.pose.orientation.x = rqx
            msg.pose.pose.orientation.y = rqy
            msg.pose.pose.orientation.z = rqz
            msg.pose.pose.orientation.w = rqw
            odom_pub.publish(msg)

            tf_msg = TransformStamped()
            tf_msg.header.stamp    = stamp
            tf_msg.header.frame_id = 'odom'
            tf_msg.child_frame_id  = 'base_link'
            tf_msg.transform.translation.x = rx
            tf_msg.transform.translation.y = ry
            tf_msg.transform.translation.z = rz
            tf_msg.transform.rotation.x = rqx
            tf_msg.transform.rotation.y = rqy
            tf_msg.transform.rotation.z = rqz
            tf_msg.transform.rotation.w = rqw
            tf_broadcaster.sendTransform(tf_msg)

        rclpy.spin_once(ros_node, timeout_sec=0)
        time.sleep(0.01)

rclpy.shutdown()
