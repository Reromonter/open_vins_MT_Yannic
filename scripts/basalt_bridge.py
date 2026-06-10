import time
import depthai as dai
import rclpy
from nav_msgs.msg import Odometry

rclpy.init()
ros_node = rclpy.create_node('basalt_bridge')
odom_pub = ros_node.create_publisher(Odometry, '/odom', 10)

# Create pipeline
with dai.Pipeline() as p:
    fps    = 10
    width  = 480
    height = 270

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
            print(f"pos: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f} | "
                  f"quat: qx={quat.qx:.3f}, qy={quat.qy:.3f}, qz={quat.qz:.3f}, qw={quat.qw:.3f}")

            msg = Odometry()
            msg.header.stamp = ros_node.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.child_frame_id = 'base_link'
            msg.pose.pose.position.x = float(pos.x)
            msg.pose.pose.position.y = float(pos.y)
            msg.pose.pose.position.z = float(pos.z)
            msg.pose.pose.orientation.x = float(quat.qx)
            msg.pose.pose.orientation.y = float(quat.qy)
            msg.pose.pose.orientation.z = float(quat.qz)
            msg.pose.pose.orientation.w = float(quat.qw)
            odom_pub.publish(msg)

        rclpy.spin_once(ros_node, timeout_sec=0)
        time.sleep(0.01)

rclpy.shutdown()
