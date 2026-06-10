import time
import depthai as dai

# Create pipeline

with dai.Pipeline() as p:
    fps = 60
    width = 640
    height = 400
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
        time.sleep(0.01)
