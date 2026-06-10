import os
import signal
import time
import numpy as np
import depthai as dai
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField

rclpy.init()
ros_node = rclpy.create_node('basalt_bridge')
odom_pub = ros_node.create_publisher(Odometry, '/odom', 10)
ptc_pub  = ros_node.create_publisher(PointCloud2, '/points', 10)


def make_pointcloud2(xyz, rgba, stamp):
    n = len(xyz)
    r = rgba[:, 0].astype(np.uint32)
    g = rgba[:, 1].astype(np.uint32)
    b = rgba[:, 2].astype(np.uint32)
    cloud = np.zeros(n, dtype=[
        ('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.uint32),
    ])
    cloud['x']   = xyz[:, 0]
    cloud['y']   = xyz[:, 1]
    cloud['z']   = xyz[:, 2]
    cloud['rgb'] = (r << 16) | (g << 8) | b
    msg = PointCloud2()
    msg.header.stamp    = stamp
    msg.header.frame_id = 'odom'
    msg.height     = 1
    msg.width      = n
    msg.fields     = [
        PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.UINT32,  count=1),
    ]
    msg.is_bigendian = False
    msg.point_step   = 16
    msg.row_step     = 16 * n
    msg.data         = cloud.tobytes()
    msg.is_dense     = True
    return msg


fps    = 10
width  = 480
height = 270

try:
    p = dai.Pipeline()

    left  = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=fps)
    right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=fps)

    imu  = p.create(dai.node.IMU)
    odom = p.create(dai.node.BasaltVIO)

    stereo = p.create(dai.node.StereoDepth)
    stereo.setSubpixel(False)
    stereo.setLeftRightCheck(False)

    left_out  = left.requestOutput((width, height))
    right_out = right.requestOutput((width, height))
    left_out.link(stereo.left)
    right_out.link(stereo.right)
    left_out.link(odom.left)
    right_out.link(odom.right)

    depth_q = stereo.depth.createOutputQueue(maxSize=4, blocking=False)

    calib  = p.getDefaultDevice().readCalibration()
    K      = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, width, height)
    fx, fy = K[0][0], K[1][1]
    cx, cy = K[0][2], K[1][2]
    uu, vv = np.meshgrid(np.arange(width), np.arange(height))

    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    imu.out.link(odom.imu)

    odomQ = odom.transform.createOutputQueue(maxSize=8, blocking=False)

    p.start()
    while p.isRunning():
        transform = odomQ.tryGet()
        if transform is not None:
            pos  = transform.getTranslation()
            quat = transform.getQuaternion()
            print(f"pos: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f} | "
                  f"quat: qx={quat.qx:.3f}, qy={quat.qy:.3f}, qz={quat.qz:.3f}, qw={quat.qw:.3f}")

            msg = Odometry()
            msg.header.stamp    = ros_node.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.child_frame_id  = 'base_link'
            msg.pose.pose.position.x    = float(pos.x)
            msg.pose.pose.position.y    = float(pos.y)
            msg.pose.pose.position.z    = float(pos.z)
            msg.pose.pose.orientation.x = float(quat.qx)
            msg.pose.pose.orientation.y = float(quat.qy)
            msg.pose.pose.orientation.z = float(quat.qz)
            msg.pose.pose.orientation.w = float(quat.qw)
            odom_pub.publish(msg)

        depth_frame = depth_q.tryGet()
        if depth_frame is None:
            rclpy.spin_once(ros_node, timeout_sec=0)
            continue

        depth = depth_frame.getFrame().astype(np.float32) / 1000.0  # mm → m
        mask  = depth > 0
        Z = depth[mask]
        X = (uu[mask] - cx) * Z / fx
        Y = (vv[mask] - cy) * Z / fy
        xyz  = np.stack([X, Y, Z], axis=1)
        rgba = np.full((len(xyz), 4), 200, dtype=np.uint8)

        print(f"Points: {len(xyz)}, Z=[{Z.min():.2f}, {Z.max():.2f}]")

        ptc_pub.publish(make_pointcloud2(xyz, rgba, ros_node.get_clock().now().to_msg()))

        rclpy.spin_once(ros_node, timeout_sec=0)
        time.sleep(0.01)

except Exception as e:
    print(f"Fatal: {e}", flush=True)
    os._exit(1)

os._exit(0)
