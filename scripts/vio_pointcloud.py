import os
import time
import threading
import numpy as np
import depthai as dai
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
import tf2_ros

rclpy.init()
ros_node = rclpy.create_node('basalt')
odom_pub  = ros_node.create_publisher(Odometry, '/odom', 10)
ptc_pub   = ros_node.create_publisher(PointCloud2, '/points', 10)
tf_broadcaster = tf2_ros.TransformBroadcaster(ros_node)


def make_odometry(transform):
    pos  = transform.getTranslation()
    quat = transform.getQuaternion()
    print(f"pos: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f} | "
          f"quat: qx={quat.qx:.3f}, qy={quat.qy:.3f}, qz={quat.qz:.3f}, qw={quat.qw:.3f}")
    stamp = ros_node.get_clock().now().to_msg()

    # VIO → ROS frame correction (observed: phys +X→VIO -X, phys -Y→VIO +Z)
    # Rotation matrix: ros_x=-vio_x, ros_y=-vio_z, ros_z=-vio_y
    # Quaternion vector part transforms identically to the position vector.
    rx  = -float(pos.x)
    ry  = -float(pos.z)
    rz  = -float(pos.y)
    rqx = -float(quat.qx)
    rqy = -float(quat.qz)
    rqz = -float(quat.qy)
    rqw =  float(quat.qw)

    msg = Odometry()
    msg.header.stamp    = stamp
    msg.header.frame_id = 'odom'
    msg.child_frame_id  = 'BASE'
    msg.pose.pose.position.x    = rx
    msg.pose.pose.position.y    = ry
    msg.pose.pose.position.z    = rz
    msg.pose.pose.orientation.x = rqx
    msg.pose.pose.orientation.y = rqy
    msg.pose.pose.orientation.z = rqz
    msg.pose.pose.orientation.w = rqw

    tf_msg = TransformStamped()
    tf_msg.header.stamp    = stamp
    tf_msg.header.frame_id = 'odom'
    tf_msg.child_frame_id  = 'BASE'
    tf_msg.transform.translation.x = rx
    tf_msg.transform.translation.y = ry
    tf_msg.transform.translation.z = rz
    tf_msg.transform.rotation.x = rqx
    tf_msg.transform.rotation.y = rqy
    tf_msg.transform.rotation.z = rqz
    tf_msg.transform.rotation.w = rqw
    tf_broadcaster.sendTransform(tf_msg)

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

def make_band_mask(shape, edges, ratios, rng):
    mask = np.zeros(shape, dtype=bool)
    for (r0, r1), ratio in zip(zip(edges[:-1], edges[1:]), ratios):
        if ratio >= 1.0:
            mask[r0:r1, :] = True
        else:
            mask[r0:r1, :] = rng.random((r1 - r0, shape[1])) < ratio
    return mask


fps    = 10
width  = 480
height = 340

band_edges = [0, height  // 3, 2 * height  // 3, height ]       # top, middle, bottom
band_ratios = [1.0, 0.6, 0.3]                                   # keep fractions per band

rng = np.random.default_rng(42)

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

                band_mask = make_band_mask(depth.shape, band_edges, band_ratios, rng)

                mask  = (depth > 0) & band_mask
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
