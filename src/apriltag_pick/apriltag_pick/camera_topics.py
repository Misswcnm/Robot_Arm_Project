"""RealSense topic names shared by Foxy and newer ROS 2 deployments."""

import os


def camera_topic_root(environ=None):
    """Return the active RealSense topic root.

    The Foxy 4.51.1 binary publishes below ``/camera`` even though its node is
    named ``/camera/camera``.  Newer wrappers publish below
    ``/camera/camera``.  An environment override keeps custom namespaces
    explicit instead of scattering distro-specific topic strings.
    """
    environ = os.environ if environ is None else environ
    configured = str(environ.get('REALSENSE_TOPIC_ROOT', '')).strip()
    if configured:
        return '/' + configured.strip('/')
    if str(environ.get('ROS_DISTRO', '')).strip().lower() == 'foxy':
        return '/camera'
    return '/camera/camera'


def camera_topic(suffix, environ=None):
    return camera_topic_root(environ) + '/' + str(suffix).lstrip('/')
