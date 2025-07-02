import tf
from cnoid.IRSLCoords import coordinates
import rospy

class TransformListener(tf.TransformListener):
    """
    A custom TransformListener class that extends tf.TransformListener to provide additional utility methods
    for working with coordinate(IRSL)
    """
    def __init__(self, *args, **kwargs):
        """
        Initializes the TransformListener instance.

        Args:
            *args: Variable length argument list passed to tf.TransformListener class.
            **kwargs: Arbitrary keyword arguments passed to tf.TransformListener class.
        """
        super().__init__(*args, **kwargs)

    def lookupCoords(self, target_frame, source_frame, rostime):
        """
        Looks up the transformation between two frames and returns it as coordinates.

        Args:
            target_frame (str): The name of the target frame.
            source_frame (str): The name of the source frame.
            rostime (rospy.Time): The time at which to perform the lookup.

        Returns:
            coordinates: A coordinates object containing the translation and rotation.
        """
        trs, quat = super().lookupTransform(target_frame, source_frame, rostime)
        return coordinates(trs, quat)

    def waitForCoords(self, origin_frame, target_frame, rostime, timeout_sec=1.0):
        """
        Waits for a transformation between two frames to become available. (Raise exception if failed)

        Args:
            origin_frame (str): The name of the origin frame.
            target_frame (str): The name of the target frame.
            rostime (rospy.Time): The time at which to wait for the transformation.
            timeout_sec (float, optional): The timeout duration in seconds. Defaults to 1.0.
        """
        self.waitForTransform(origin_frame, target_frame, rostime, rospy.Duration(timeout_sec))

    def tryWaiting(self, origin_frame, target_frame, rostime, timeout_sec=1.0):
        """
        Attempts to wait for a transformation between two frames to become available. (Return True if the transformation was found)

        Args:
            origin_frame (str): The name of the origin frame.
            target_frame (str): The name of the target frame.
            rostime (rospy.Time): The time at which to wait for the transformation.
            timeout_sec (float, optional): The timeout duration in seconds. Defaults to 1.0.

        Returns:
            bool: True if the transformation becomes available, False otherwise.
        """
        try:
            self.waitForTransform(origin_frame, target_frame, rostime, rospy.Duration(timeout_sec))
        except Exception as e:
            rospy.logwarn('{}'.format(e))
            return False
        return True

class TransformBroadcaster(tf.TransformBroadcaster):
    """
    A custom TransformBroadcaster class that extends tf.TransformBroadcaster to provide additional utility methods
    for working with coordinate(IRSL)
    """
    def __init__(self, *args, **kwargs):
        """
        Initializes the TransformListener instance.

        Args:
            *args: Variable length argument list passed to tf.TransformBroadcaster class.
            **kwargs: Arbitrary keyword arguments passed to tf.TransformBroadcaster class.
        """
        super().__init__(*args, **kwargs)

    def sendCoords(self, coords, rostime, child, parent):
        """
        Sends the coordinates as TF

        Args:
            coords (object): An object containing `pos` (position as a list of 3 floats)
                and `quaternion` (rotation as a list of 4 floats).
            rostime (rospy.Time): The timestamp for the transform.
            child (str): The child frame ID.
            parent (str): The parent frame ID.

        Returns:
            bool: True if the transform was successfully sent, False otherwise.
        """
        pos = coords.pos
        translation = [pos[0], pos[1], pos[2]]
        q = coords.quaternion
        rotation = [q[0], q[1], q[2], q[3]]
        return self.sendTransform(translation, rotation, rostime, child, parent)
