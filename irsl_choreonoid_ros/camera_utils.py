import image_geometry
from sensor_msgs.msg import RegionOfInterest
import numpy as np
import irsl_choreonoid.make_shapes as mkshapes

class PinholeCameraModel(image_geometry.PinholeCameraModel):
    def __init__(self):
        pass

    def generatePointsRaw(self, depthImageCv, transform=None, colorImageCv=None, threshold=0.1, ROI=None):
        """
        Generates 3D points and corresponding colors from a depth image using a pinhole camera model.

        Args:
            depthImageCv (numpy.ndarray): Depth image in CV format, where depth values are in millimeters.
            transform (optional): Transformation object to apply to the generated 3D points. Defaults to None.
            colorImageCv (numpy.ndarray, optional): Color image in CV format, used to extract color information for points. Defaults to None.
            threshold (float, optional): Minimum depth value to consider. Points with depth below this value are ignored. Defaults to 0.1.
            ROI (sensor_msgs.msg.RegionOfInterest or tuple, optional): Region of Interest (ROI) object specifying the area of the image to process. Defaults to None.

        Returns:
            (points, colors) : A tuple containing:
                - points (list of numpy.ndarray): List of 3D points in the camera coordinate system.
                - colors (list of numpy.ndarray): List of color values corresponding to the 3D points. Empty if `colorImageCv` is None.
        """
        cx = self.cx()
        cy = self.cy()
        fx = self.fx()
        fy = self.fy()
        height, width = depthImageCv.shape[0:2]
        # mm -> mに単位変換
        depthImageCv = depthImageCv * 0.001
        points = []
        colors = []
        # データ作成範囲指定
        y_start = 0      if ROI is None else ROI.y_offset
        y_end   = height if ROI is None else ROI.y_offset + ROI.height
        x_start = 0      if ROI is None else ROI.x_offset
        x_end   = width  if ROI is None else ROI.x_offset + ROI.width
        #
        for v in range(y_start, y_end):
            for u in range(x_start, x_end):
                # 深度値を取得ししきい値以下は無視する
                z = depthImageCv[v, u]
                if z < threshold:
                    continue
                # ピンホールカメラモデルに当てはめて内部パラメータよりx,y座標値を計算
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                #
                p = np.array([x, y, z])
                if transform is not None:
                    transform.transformVector(p)
                #
                if colorImageCv is not None:
                    # 色データを取得(値域が[0,255]から@)
                    color = colorImageCv[v, u] / 255.0
                    # 結果を格納
                    colors.append(color)
                points.append( p )
        return points, colors

    def generatePoints(self, depthImageCv, transform=None, colorImageCv=None, threshold=0.1, ROI=None, **kwargs):
        """
        Generates SceneNode of 3D points from a depth image and optionally associates colors with the points.

        Args:
            depthImageCv (numpy.ndarray): The depth image in OpenCV format.
            transform (coordinates, optional): A transformation matrix to apply to the points. Defaults to None.
            colorImageCv (numpy.ndarray, optional): The color image in OpenCV format to associate colors with the points. Defaults to None.
            threshold (float, optional): The depth threshold to filter points. Defaults to 0.1.
            ROI (sensor_msgs.msg.RegionOfInterest or tuple, optional): A region of interest specified as (x, y, width, height). Defaults to None.
            **kwargs: Additional keyword arguments to pass to the `mkshapes.makePoints` function.

        Returns:
            object: A points object created by `mkshapes.makePoints`, containing the generated 3D points and optionally their associated colors.
        """
        points, colors = self.generatePointsRaw(depthImageCv, transform, colorImageCv, threshold, ROI)
        if len(colors) == 0:
            colors = None
        return mkshapes.makePoints(points, colors=colors, **kwargs)

def cameraModelFromCameraInfo(camera_info):
    """
    Generate Camera-model from a CameraInfo message

    Args:
        camera_info (sensor_msgs.msg.CameraInfo): The CameraInfo message

    Returns:
        PinholeCameraModel: An instance of PinholeCameraModel initialized with 
            the parameters from the provided CameraInfo message.
    """
    pcm = PinholeCameraModel()
    pcm.from_camera_info(camera_info)
    return pcm
