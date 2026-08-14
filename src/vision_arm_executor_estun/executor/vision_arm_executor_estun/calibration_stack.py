"""Run ESTUN driver, D455 and interactive hand-eye calibration together."""

import argparse
import os
import signal
import subprocess
import time

from . import handeye_calibration


ESTUN_POSE_SERVICE = '/estun_codroid/get_pose'
CAMERA_IMAGE_TOPIC = '/camera/camera/color/image_raw'
CAMERA_INFO_TOPIC = '/camera/camera/color/camera_info'


def driver_command(model, robot_ip, use_fake_hardware=False):
    del model
    if use_fake_hardware:
        raise ValueError('Codroid bridge does not provide fake hardware')
    return [
        'ros2', 'launch', 'estun_codroid_bridge',
        'codroid_bridge.launch.py',
        'robot_ip:=' + robot_ip,
        'robot_port:=9000',
    ]


def camera_command():
    return [
        'ros2', 'launch', 'realsense2_camera', 'rs_launch.py',
        'pointcloud.enable:=true',
    ]


def _ros_names(command):
    try:
        result = subprocess.run(
            command, check=False, capture_output=True, text=True,
            timeout=4.0)
    except (OSError, subprocess.TimeoutExpired):
        return set()
    if result.returncode != 0:
        return set()
    return {line.strip() for line in result.stdout.splitlines() if line.strip()}


def wait_ready(driver, camera, timeout):
    deadline = time.monotonic() + timeout
    last_missing = None
    while time.monotonic() < deadline:
        if driver is not None and driver.poll() is not None:
            raise RuntimeError(
                'ESTUN driver exited during startup (code=%s)' %
                driver.returncode)
        if camera is not None and camera.poll() is not None:
            raise RuntimeError(
                'RealSense driver exited during startup (code=%s)' %
                camera.returncode)
        services = _ros_names(['ros2', 'service', 'list'])
        topics = _ros_names(['ros2', 'topic', 'list'])
        missing = []
        if ESTUN_POSE_SERVICE not in services:
            missing.append(ESTUN_POSE_SERVICE)
        for topic in (CAMERA_IMAGE_TOPIC, CAMERA_INFO_TOPIC):
            if topic not in topics:
                missing.append(topic)
        if not missing:
            print('ESTUN Codroid get_pose 和 D455 图像/内参均已就绪。')
            return
        current = tuple(missing)
        if current != last_missing:
            print('等待: ' + ', '.join(missing))
            last_missing = current
        time.sleep(0.5)
    raise RuntimeError(
        'startup timed out; unavailable: %s' % ', '.join(last_missing or []))


def stop_process(process, name):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=8.0)
        return
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=3.0)
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                print('%s did not stop cleanly; pid=%d' %
                      (name, process.pid))


def parser():
    result = argparse.ArgumentParser(
        description='Start ESTUN + D455 + hand-eye calibration in one terminal')
    result.add_argument('--model', default='ER20-1780-A6')
    result.add_argument(
        '--robot-ip', default=os.environ.get('ESTUN_ROBOT_IP', ''))
    result.add_argument('--use-fake-hardware', action='store_true')
    result.add_argument('--startup-timeout-sec', type=float, default=60.0)
    result.add_argument('--no-activate', action='store_true')
    return result


def main(args=None):
    options, calibration_args = parser().parse_known_args(args)
    if not options.use_fake_hardware and not options.robot_ip:
        raise SystemExit(
            '--robot-ip is required (or export ESTUN_ROBOT_IP)')

    driver = None
    camera = None
    try:
        services = _ros_names(['ros2', 'service', 'list'])
        if ESTUN_POSE_SERVICE in services:
            print('[ESTUN] 复用已运行的 %s' % ESTUN_POSE_SERVICE)
        else:
            print('[ESTUN] ' + ' '.join(driver_command(
                options.model, options.robot_ip, options.use_fake_hardware)))
            driver = subprocess.Popen(
                driver_command(
                    options.model, options.robot_ip,
                    options.use_fake_hardware),
                start_new_session=True)
        topics = _ros_names(['ros2', 'topic', 'list'])
        if (CAMERA_IMAGE_TOPIC in topics and CAMERA_INFO_TOPIC in topics):
            print('[D455] 复用已运行的相机图像和内参话题')
        else:
            print('[D455] ' + ' '.join(camera_command()))
            camera = subprocess.Popen(
                camera_command(), start_new_session=True)
        wait_ready(driver, camera, options.startup_timeout_sec)

        tool_args = []
        if not options.no_activate:
            tool_args.append('--activate')
        tool_args.extend(calibration_args)
        handeye_calibration.main(tool_args)
    except KeyboardInterrupt:
        print('\n手眼标定已中止。')
    except RuntimeError as error:
        raise SystemExit('手眼标定栈启动失败: %s' % error)
    finally:
        print('正在关闭 D455 和 ESTUN 驱动...')
        stop_process(camera, 'RealSense')
        stop_process(driver, 'ESTUN driver')


if __name__ == '__main__':
    main()
