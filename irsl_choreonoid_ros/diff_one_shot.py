class RosDeviceBase(object):
    def __init__(self, dev_dict, robot=None, node=None, queueSize=10):
        super().__init__()
        self._node = node
        self.robot_callback = None
        self.__robot = robot
        self.topic = dev_dict['topic']
        self.name  = dev_dict['name']
        ## TODO: implement rate
        if 'rate' in dev_dict:
            self.rate = dev_dict['rate']
        else:
            self.rate = None
        ## TODO: search device in robot
        self.msg_time = None
        self.current_msg = None
        self.timeout = None
        self.sub = None
        self.queueSize = queueSize

    @property
    def connected(self):
        if self.sub is not None and self.sub.get_num_connections() > 0:
            return True
        return False

    def parseType(self, type_str):
        if type(type_str) is str:
            tp = type_str.split('/')
            exec('from {}.msg import {}'.format(tp[0], tp[1]), locals(), globals())
            self.msg = eval('{}'.format(tp[1]))
        else:
            self.msg = type_str
        ## exec('class {}Wrapped({}):\n  __slots__ = ("header")')
        ## self.msg_wrapped = eval('{}Wrapped'.format(tp[1]))
    def subscribe(self):
        self.sub = self._node.create_subscription(self.msg, self.topic, self.callback, self.queueSize)

    def callback(self, msg):
        #if not hasattr(msg, 'header'):
        #    setattr(msg, 'header', std_msgs_header(stamp=TODO_FIX_ROS1_ROSPY.get_rostime(), frame_id='add_by_ri'))
        #    ## msg.header = std_msgs_header(stamp=TODO_FIX_ROS1_ROSPY.get_rostime(), frame_id='add_by_ri')
        self.msg_time = self._node.get_clock().now()
        if self.robot_callback is not None:
            self.robot_callback(msg)
        self.current_msg = msg

    def returnData(self):
        return self.current_msg

    def data(self, clear=False):
        res = self.returnData()
        tm = self.msg_time
        if clear:
            self.msg_time = None
            self.current_msg = None
        return res

    ## protected functions
    def _pre_wait(self, timeout):
        if timeout is None:
            self.timeout = None
        else:
            self.timeout = self._node.get_clock().now() + Duration(seconds=timeout)

    def _pre_wait_next(self, timeout):
        self._pre_wait(timeout)
        self.msg_time = None
        self.current_msg = None

    def _fetch_data(self, clear=False):
        while ( self.timeout is None ) or ( self.timeout >=  self._node.get_clock().now() ):
            if self.current_msg is not None:
                return self.data(clear)
            else:
                time.sleep(0.002) ## TODO

    def _unsubscribe(self):
        ## unsubscribe
        self.sub.unregister()
        self.sub = None

    def _remove(self):
        self.msg_time    = None
        self.timeout     = None
        self.current_msg = None
        self._unsubscribe() ## needed ??

    ##
    def waitData(self, timeout=None, clear=False):
        self._pre_wait(timeout)
        return self._fetch_data(clear)

    def waitNextData(self, timeout=None, clear=False):
        self._pre_wait_next(timeout)
        return self._fetch_data(clear)
    ##
    @property
    def robot(self):
        return self.__robot
#    @robot.setter
#    def robot(self, in_robot):
#        self.__robot = in_robot

class OneShotSubscriber(RosDeviceBase):
    """
    Subscribe just N messages

    """
    def __init__(self, topic, msg, size=1, **kwargs):
        """"
        Args:
            topic ( str ) : Name of Topic
            msg ( class ) : Class instance of message to subscribe
            size (int, default = 1 ) : Size of messages to be subscribed

        """
        self.topic = topic
        self.msg = msg
        self.size = size
        self.msg_time = None
        self.current_msg = None
        self.results = [] ## size
        self.sub = rospy.Subscriber(self.topic, self.msg, self.callback, **kwargs)

    def callback(self, msg):
        self.msg_time = rospy.get_rostime()
        self.current_msg = msg
        self.results.append(msg) ##
        if len(self.results) >= self.size:
            self._unsubscribe()

    def waitResults(self, timeout=None):
        """"Waiting N results

        Args:
            topic ( str ) : Name of Topic
            msg ( class ) : Class instance of message to subscribe
            size (int, default = 1 ) : Size of messages to be subscribed

        Returns:
            list( ros_message ) : Subscribed messages. If timeout occurs, None is returned.

        """
        self._pre_wait(timeout)
        while ( self.timeout is None ) or ( self.timeout >= rospy.get_rostime() ):
            if len(self.results) >= self.size:
                return self.results
            else:
                rospy.sleep(0.002)


class OneShotSyncSubscriber(RosDeviceBase):
    """
    Subscribe just N sync messages

    """
    def __init__(self, topics, msgs, size=1, sync_message_queue_size=100, sync_slop=0.02):
        """"
        Args:
            topics ( list of str ) : Name of Topic
            msgs ( list of class ) : Class instance of message to subscribe
            size (int, default = 1 ) : Size of messages to be subscribed
            sync_message_queue_size ( int, default = 100 ) : Size of message queue
            sync_slop ( float, default = 0.02 ) :  Delay with which messages can be synchronized in seconds).

        """
        self.topics = topics
        self.msgs = msgs
        self.size = size
        self.msg_time = None
        self.current_msg = None
        self.results = [] ## size
        self.filter_subs = [ message_filters.Subscriber(topic, msg) for topic, msg in zip(topics, msgs) ]
        self.ts = message_filters.ApproximateTimeSynchronizer(self.filter_subs, sync_message_queue_size, sync_slop)
        self.ts.registerCallback(self.callback)

    def callback(self, *msgs):
        self.msg_time = rospy.get_rostime()
        self.current_msg = msgs
        self.results.append(msgs) ##
        if len(self.results) >= self.size:
            _ = [sub.sub.unregister() for sub in self.filter_subs]

    def waitResults(self, timeout=None):
        """"Waiting N results

        Args:
            timeout ( float, default = None ) : Time for timeout

        Returns:
            None

        """
        self._pre_wait(timeout)
        while ( self.timeout is None ) or ( self.timeout >= rospy.get_rostime() ):
            if len(self.results) >= self.size:
                return self.results
            else:
                rospy.sleep(0.002)
