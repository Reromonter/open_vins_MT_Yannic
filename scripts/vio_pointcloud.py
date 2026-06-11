import os
import time
import threading
import numpy as np
import depthai as dai
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField

rclpy.init()
ros_node = rclpy.create_node('basalt')
odom_pub = ros_node.create_publisher(Odometry, '/gazebo/odom', 10)
ptc_pub  = ros_node.create_publisher(PointCloud2, '/depth_camera/points', 10)


def make_odometry(transform):
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
    return msg


def make_pointcloud2(xyz, stamp):
    Z = xyz[:, 2]
    print(f"Points: {len(xyz)}, Z=[{Z.min():.2f}, {Z.max():.2f}]")
    n = len(xyz)
    cloud = np.zeros(n, dtype=[
        ('x', np.float32), ('y', np.float32), ('z', np.float32),
    ])
    cloud['x'] = xyz[:, 0]
    cloud['y'] = xyz[:, 1]
    cloud['z'] = xyz[:, 2]
    msg = PointCloud2()
    msg.header.stamp    = stamp
    msg.header.frame_id = 'depth_camera_optical_frame'
    msg.height     = 1
    msg.width      = n
    msg.fields     = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step   = 12
    msg.row_step     = 12 * n
    msg.data         = cloud.tobytes()
    msg.is_dense     = True
    return msg


fps    = 10
width  = 240
height = 135

print(f"[vio_pointcloud] fps={fps}  resolution={width}x{height}  imu_rate=200 Hz  topics=/odom /points", flush=True)

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

    # Check ~100x per frame interval. Should be enough to catch new data without busy-waiting.
    idle_sleep = 1.0 / (fps * 100)

    def odom_loop():
        while p.isRunning():
            transform = odomQ.tryGet()
            if transform is not None:
                odom_pub.publish(make_odometry(transform))
            else:
                # No transform available. Sleep so Python can run depth_loop —
                # without this, the loop spins millions of times/sec and
                # Python's GIL never gets a chance to switch to the other thread.
                time.sleep(idle_sleep)

    def depth_loop():
        while p.isRunning():
            depth_frame = depth_q.tryGet()
            if depth_frame is not None:
                depth = depth_frame.getFrame().astype(np.float32) / 1000.0  # mm → m
                mask  = depth > 0
                Z = depth[mask]
                X = (uu[mask] - cx) * Z / fx
                Y = (vv[mask] - cy) * Z / fy
                xyz = np.stack([X, Y, Z], axis=1)

                ptc_pub.publish(make_pointcloud2(xyz, ros_node.get_clock().now().to_msg()))
                rclpy.spin_once(ros_node, timeout_sec=0)
            else:
                # Same as above: sleep so odom_loop gets CPU time while
                # we wait for the next depth frame (which only arrives at 10 fps).
                time.sleep(idle_sleep)

    p.start()
    t_odom  = threading.Thread(target=odom_loop,  daemon=True)
    t_depth = threading.Thread(target=depth_loop, daemon=True)
    t_odom.start()
    t_depth.start()
    t_odom.join()
    t_depth.join()

except Exception as e:
    print(f"Fatal: {e}", flush=True)
    os._exit(1)

os._exit(0)
