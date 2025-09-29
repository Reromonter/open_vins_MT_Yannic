#!/usr/bin/env python3
import os
import time
import psutil
import rclpy
from rclpy.node import Node

def find_process_for_token(token: str):
    # token can be a ROS 2 node name like /ov_msckf/run_subscribe_msckf; use the basename
    base = token.strip().split('/')[-1]
    for p in psutil.process_iter(attrs=['pid', 'name', 'cmdline']):
        try:
            name = (p.info.get('name') or '').lower()
            cmd = ' '.join(p.info.get('cmdline') or []).lower()
            if base.lower() in name or base.lower() in cmd:
                return p
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    return None

class PidRos2(Node):
    def __init__(self):
        super().__init__('pid_ros2')
        self.declare_parameter('nodes', '/ov_msckf/run_subscribe_msckf')
        self.declare_parameter('output', '/catkin_ws/logging_files/psutil_log.txt')
        nodes_csv = self.get_parameter('nodes').get_parameter_value().string_value
        self.save_path = self.get_parameter('output').get_parameter_value().string_value

        self.tokens = [t.strip() for t in nodes_csv.split(',') if t.strip()]
        self.get_logger().info(f'process tokens: {self.tokens} ({len(self.tokens)} total)')
        self.get_logger().info(f'save path: {self.save_path}')

        # ensure dir exists
        os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
        self.file = open(self.save_path, 'w')
        
        # Use spaces instead of commas for header
        header = '# timestamp(s) summed_cpu_perc summed_mem_perc summed_threads'
        for t in self.tokens:
            header += f' {t}_cpu_perc {t}_mem_perc {t}_threads'
        self.file.write(header + '\n')
        self.file.flush()

        # warm-up psutil cpu_percent
        self.proc_list = [find_process_for_token(t) for t in self.tokens]
        for p in self.proc_list:
            try:
                if p: p.cpu_percent(interval=None)
            except Exception:
                pass

        self.timer = self.create_timer(0.1, self.tick)

    def tick(self):
        # refresh mapping if any process went away
        for i, p in enumerate(self.proc_list):
            if p is None or not p.is_running():
                self.proc_list[i] = find_process_for_token(self.tokens[i])

        # first sample (non-blocking)
        snapshot = []
        for p in self.proc_list:
            try:
                if p is None:
                    snapshot.append((0.0, 0.0, 0))
                else:
                    cpu = p.cpu_percent(interval=None)
                    mem = p.memory_percent()
                    thr = p.num_threads()
                    snapshot.append((cpu, mem, thr))
            except Exception:
                snapshot.append((0.0, 0.0, 0))

        # log
        sum_cpu = sum(x[0] for x in snapshot)
        sum_mem = sum(x[1] for x in snapshot)
        sum_thr = sum(x[2] for x in snapshot)
        #self.get_logger().info(f'cpu%={sum_cpu:.3f} | mem%={sum_mem:.3f} | threads={sum_thr}')
        
        # Use spaces instead of commas for data lines
        line = f'{time.time():.8f} {sum_cpu:.3f} {sum_mem:.3f} {sum_thr:d}'
        for cpu, mem, thr in snapshot:
            line += f' {cpu:.3f} {mem:.3f} {thr:d}'
        self.file.write(line + '\n')
        self.file.flush()

    def destroy_node(self):
        try:
            if hasattr(self, 'file'):
                self.file.close()
        finally:
            super().destroy_node()

def main():
    rclpy.init()
    node = PidRos2()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()