import os
import signal
import time
import numpy as np
import depthai as dai
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField

# BasaltVIO::stop() segfaults in depthai on teardown — always exit hard.
signal.signal(signal.SIGINT,  lambda *_: os._exit(0))
signal.signal(signal.SIGTERM, lambda *_: os._exit(0))

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


fps    = 60
width  = 640
height = 400

try:
    p = dai.Pipeline()

    left  = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=fps)
    right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=fps)
    color = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)

    imu  = p.create(dai.node.IMU)
    odom = p.create(dai.node.BasaltVIO)

    stereo = p.create(dai.node.StereoDepth)

    left_out  = left.requestOutput((width, height))
    right_out = right.requestOutput((width, height))
    left_out.link(stereo.left)
    right_out.link(stereo.right)

    colorOut = color.requestOutput((640, 400), type=dai.ImgFrame.Type.RGB888i,
                                   resizeMode=dai.ImgResizeMode.CROP, enableUndistortion=True)

    pc = p.create(dai.node.PointCloud)
    pc.initialConfig.setLengthUnit(dai.LengthUnit.METER)

    platform = p.getDefaultDevice().getPlatform()
    if platform == dai.Platform.RVC4:
        imageAlign = p.create(dai.node.ImageAlign)
        stereo.depth.link(imageAlign.input)
        colorOut.link(imageAlign.inputAlignTo)
        imageAlign.outputAligned.link(pc.inputDepth)
    else:
        colorOut.link(stereo.inputAlignTo)
        stereo.depth.link(pc.inputDepth)

    colorOut.link(pc.inputColor)

    q = pc.outputPointCloud.createOutputQueue(maxSize=4, blocking=False)

    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    left.requestOutput((width, height)).link(odom.left)
    right.requestOutput((width, height)).link(odom.right)
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

        pcd = q.tryGet()
        if pcd is None:
            rclpy.spin_once(ros_node, timeout_sec=0)
            continue

        if pcd.isColor():
            xyz, rgba = pcd.getPointsRGB()
        else:
            xyz  = pcd.getPoints()
            rgba = np.full((len(xyz), 4), 200, dtype=np.uint8)

        print(f"Points: {len(xyz)}, {pcd.getWidth()}x{pcd.getHeight()}, "
              f"color={pcd.isColor()}, Z=[{pcd.getMinZ():.2f}, {pcd.getMaxZ():.2f}]")

        ptc_pub.publish(make_pointcloud2(xyz, rgba, ros_node.get_clock().now().to_msg()))

        rclpy.spin_once(ros_node, timeout_sec=0)
        time.sleep(0.01)

except Exception as e:
    print(f"Fatal: {e}", flush=True)
    os._exit(1)

os._exit(0)
