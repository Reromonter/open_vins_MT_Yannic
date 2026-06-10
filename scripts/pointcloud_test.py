#!/usr/bin/env python3
"""PointCloud bridge: publishes colorized point cloud from stereo depth + RGB to /points."""

import numpy as np
import depthai as dai
import rclpy
from sensor_msgs.msg import PointCloud2, PointField

rclpy.init()
node = rclpy.create_node('pointcloud_bridge')
pub  = node.create_publisher(PointCloud2, '/points', 10)


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


pipeline = dai.Pipeline()

left  = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
color = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)

stereo = pipeline.create(dai.node.StereoDepth)
left.requestFullResolutionOutput().link(stereo.left)
right.requestFullResolutionOutput().link(stereo.right)

colorOut = color.requestOutput((640, 400), type=dai.ImgFrame.Type.RGB888i,
                               resizeMode=dai.ImgResizeMode.CROP, enableUndistortion=True)

pc = pipeline.create(dai.node.PointCloud)
pc.initialConfig.setLengthUnit(dai.LengthUnit.METER)

platform = pipeline.getDefaultDevice().getPlatform()
if platform == dai.Platform.RVC4:
    imageAlign = pipeline.create(dai.node.ImageAlign)
    stereo.depth.link(imageAlign.input)
    colorOut.link(imageAlign.inputAlignTo)
    imageAlign.outputAligned.link(pc.inputDepth)
else:
    colorOut.link(stereo.inputAlignTo)
    stereo.depth.link(pc.inputDepth)

colorOut.link(pc.inputColor)

q = pc.outputPointCloud.createOutputQueue(maxSize=4, blocking=False)

with pipeline:
    pipeline.start()
    while pipeline.isRunning():
        pcd = q.get()
        if pcd is None:
            rclpy.spin_once(node, timeout_sec=0)
            continue

        if pcd.isColor():
            xyz, rgba = pcd.getPointsRGB()
        else:
            xyz  = pcd.getPoints()
            rgba = np.full((len(xyz), 4), 200, dtype=np.uint8)

        print(f"Points: {len(xyz)}, {pcd.getWidth()}x{pcd.getHeight()}, "
              f"color={pcd.isColor()}, Z=[{pcd.getMinZ():.2f}, {pcd.getMaxZ():.2f}]")

        pub.publish(make_pointcloud2(xyz, rgba, node.get_clock().now().to_msg()))
        rclpy.spin_once(node, timeout_sec=0)

rclpy.shutdown()
