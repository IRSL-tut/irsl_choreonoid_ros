import rospy
import roslib.packages

from urllib.parse import urlparse
from irsl_choreonoid.cnoid_util import parseURL

def parseURLROS(url):
    """parseURLROS

    Refere parseURL in irsl_choreonoid.cnoid_util

    Args:
        url (str): url

    Returns:
        str: Absolute path

    Examples:
        >>> parseURLROS('package://ros_package_name/dir/file')
        /catkin_ws/install/share/ros_package_name/dir/file

    """
    try:
        return parseURL(url)
    except Exception as e:
        res = urlparse(url)
        if res.scheme == 'package':
            return roslib.packages.get_pkg_dir(res.netloc) + res.path
        elif res.scheme == 'rosparam':
            raise SyntaxError('not implemented scheme {} / {}'.format(res.scheme, url))
            if res.netloc == '':
                return rospy.get_param(res.path)
            elif res.netloc == '.':
                ## relative
                path = res.path
                if path[0] == '/':
                    path = path[1:]
                return rospy.get_param(path)
            elif res.netloc == '~':
                path = res.path
                if path[0] == '/':
                    path = path[1:]
                return rospy.get_param(res.netloc + path)
        else:
            raise e

def initializeROS(node_name=None, MASTER_URI=None, MASTER=None, MASTER_PORT=11311, IP=None, HOSTNAME=None, **kwargs):
    """
    """
    #
    if MASTER is not None:
        if 'ROS_MASTER_URI' ins in os.environ:
            printf(f'ROS_MASTER_URI({os.environ["ROS_MASTER_URI"]}) is overwritten by {MASTER}')
        os.environ['ROS_MASTER_URI'] = 'http://{}:{}'.format(MASTER, MASTER_PORT)
    elif MASTER_URI is not None:
        if 'ROS_MASTER_URI' ins in os.environ:
            printf(f'ROS_MASTER_URI({os.environ["ROS_MASTER_URI"]}) is overwritten by {MASTER}')
        os.environ['ROS_MASTER_URI'] = MASTER_URI
    if IP is not None:
        if 'ROS_IP' ins in os.environ:
            printf(f'ROS_IP({os.environ["ROS_IP"]}) is overwritten by {IP}')
        os.environ['ROS_IP']         = IP
    if HOSTNAME is None:
        HOSTNAME = IP
    if HOSTNAME is not None:
        if 'ROS_HOSTNAME' ins in os.environ:
            printf(f'ROS_HOSTNAME({os.environ["ROS_HOSTNAME"]}) is overwritten by {HOSTNAME}')
        os.environ['ROS_HOSTNAME']   = HOSTNAME
    #
    if node_name is not None:
        rospy.init_node(node_name, **kwargs)
