from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
import math
import numpy as np
import irsl_choreonoid.make_shapes as mkshapes

#
from sensor_msgs.msg import LaserScan

#from std_msgs.msg import Header
# isnan

def unpack_rgb(fval):
    # cast float32 to int so that bitwise operations are possible
    s = struct.pack('>f', fval)
    i = struct.unpack('>l', s)[0]
    # you can get back the float value by the inverse operations
    pack = ctypes.c_uint32(i).value
    r = (pack & 0x00FF0000)>> 16
    g = (pack & 0x0000FF00)>> 8
    b = (pack & 0x000000FF)
    return (r, g, b)

def convertPointCloud2ToPointsRaw(pc2msg, skip_nans=True, nanValue=None, ROI=None, addColor=True):
    """
    Converts a PointCloud2 ROS message into raw points and optionally extracts colors.

    Args:
        pc2msg (sensor_msgs.msg.PointCloud2): The PointCloud2 message to be converted.
        skip_nans (bool, optional): If True, skips points with NaN values. Defaults to True.
        nanValue (float, optional): Value to replace NaNs with if `skip_nans` is False. Defaults to None.
        ROI (tuple, optional): Region of interest specified as (x, y, width, height). If provided, only points within this ROI are processed. Defaults to None.
        addColor (bool, optional): If True, extracts RGB color information from the point cloud if available. Defaults to True.

    Returns:
        tuple: A tuple containing:
            - points (list of numpy.ndarray): A list of 3D points as numpy arrays [x, y, z].
            - colors (list of numpy.ndarray): A list of RGB colors as numpy arrays [r, g, b]. Empty if `addColor` is False or no color information is available.

    Raises:
        ValueError: If the ROI is invalid or out of bounds.
    """
    points = []
    colors = []
    #
    uvs = None
    if ROI is not None:
        uvs = []
        for u in range(ROI[0], ROI[0]+ROI[2]):
            for v in range(ROI[1], ROI[1]+ROI[3]):
                uvs.append( (u, v) )
    #
    _addColor = False
    if addColor:
        if len(pc2msg.fields) > 3:
            if pc2msg.fields[3].name == 'rgb':
                _addColor = True
    #
    gen = pc2.read_points(pc2msg, skip_nans=skip_nans, uvs=uvs)
    #
    for data in gen:
        points.append( np.array([ data[0], data[1], data[2] ]) )
        if _addColor:
            r, g, b = unpack_rgb(data[3])
            colors.append( np.array([r, g, b]) )
    return points, colors

def convertPointCloud2ToPoints(pc2msg, skip_nans=True, nanValue=None, ROI=None, addColor=True, **kwargs):
    """
    Converts a PointCloud2 ROS message to SgPoint

    Args:
        pc2msg (sensor_msgs.msg.PointCloud2): The PointCloud2 message to be converted.
        skip_nans (bool, optional): If True, skips points with NaN values. Defaults to True.
        nanValue (float, optional): Value to replace NaNs with, if `skip_nans` is False. Defaults to None.
        ROI (tuple, optional): Region of Interest specified as a tuple (xmin, xmax, ymin, ymax, zmin, zmax). 
            If provided, only points within this region are included. Defaults to None.
        addColor (bool, optional): If True, extracts color information from the PointCloud2 message. Defaults to True.
        **kwargs: Additional keyword arguments passed to the `mkshapes.makePoints` function.

    Returns:
        object: A point cloud shape created using the processed points and colors.
    """
    points, colors = convertPointCloud2ToPointsRaw(pc2msg, skip_nans=skip_nans, nanValue=nanValue, ROI=ROI, addColor=addColor)
    if len(colors) == 0:
        colors = None
    return mkshapes.makePoints(points, colors=colors, **kwargs)

def convertLaserScanToPointsRaw(lsmsg, useIntensities=False, maxIntensity=1.0, maxColor=None, minColor=None, skip_inf=True):
    """
    Converts a LaserScan ROS message into a list of 2D points and optionally their corresponding colors.

    Args:
        lsmsg (LaserScan): The LaserScan message containing range and intensity data.
        useIntensities (bool, optional): Whether to use intensity values to compute colors. Defaults to False.
        maxIntensity (float, optional): The maximum intensity value for normalization. Defaults to 1.0.
        maxColor (numpy.ndarray, optional): The RGB color corresponding to maximum intensity. Defaults to [0., 1., 0.] (green).
        minColor (numpy.ndarray, optional): The RGB color corresponding to minimum intensity. Defaults to [1., 0., 0.] (red).
        skip_inf (bool, optional): Whether to skip points with infinite range values. Defaults to True.

    Returns:
        tuple: A tuple containing:
            - points (list of numpy.ndarray): A list of 2D points in the form [x, y, z=0].
            - colors (list of numpy.ndarray): A list of RGB colors corresponding to the points (empty if `useIntensities` is False).
    """
    points = []
    colors = []
    # tmp0 = math.ceil( (lsmsg.angle_max - lsmsg.angle_min) / lsmsg.angle_increment )
    # tmp1 = lsmsg.ranges
    if useIntensities:
        if maxColor is None:
            maxColor = np.array([0., 1., 0.])
        if minColor is None:
            minColor = np.array([1., 0., 0.])
    #
    for i in range(len(lsmsg.ranges)):
        angle = lsmsg.angle_min + lsmsg.angle_increment*i
        dist = lsmsg.ranges[i]
        if not skip_inf or not math.isinf(dist):
            points.append( np.array([ dist*math.cos(angle), dist*math.sin(angle), 0 ]) )
            if useIntensities:
                intent = lsmsg.intensities[i]/maxIntensity
                colors.append( intent * maxColor + (1 - intent) * minColor )
    #
    return points, colors

def convertLaserScanToPoints(lsmsg, useIntensities=False, maxIntensity=1.0, maxColor=None, minColor=None, skip_inf=True, **kwargs):
    """
    Converts a LaserScan ROS message to SgPoint

    Args:
        lsmsg (sensor_msgs.msg.LaserScan): The LaserScan message to be converted.
        useIntensities (bool, optional): Whether to use intensity values for coloring the points. Defaults to False.
        maxIntensity (float, optional): The maximum intensity value for scaling colors. Defaults to 1.0.
        maxColor (tuple or list, optional): The RGB color corresponding to the maximum intensity. Defaults to None.
        minColor (tuple or list, optional): The RGB color corresponding to the minimum intensity. Defaults to None.
        skip_inf (bool, optional): Whether to skip points with infinite range values. Defaults to True.
        **kwargs: Additional keyword arguments to be passed to the `mkshapes.makePoints` function.

    Returns:
        object: A points object created by `mkshapes.makePoints`, containing the 3D points and optional colors.
    """
    points, colors = convertLaserScanToPointsRaw(lsmsg, useIntensities=useIntensities, maxIntensity=maxIntensity,
                                                 maxColor=maxColor, minColor=minColor, skip_inf=skip_inf)
    if len(colors) == 0:
        colors = None
    return mkshapes.makePoints(points, colors=colors, **kwargs)
