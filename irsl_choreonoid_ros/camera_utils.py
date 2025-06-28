import image_geometry
from sensor_msgs.msg import RegionOfInterest
import numpy as np
import irsl_choreonoid.make_shapes as mkshapes

class PinholeCameraModel(image_geometry.PinholeCameraModel):
    def __init__(self):
        pass

    def generatePointsRaw(self, depthImageCv, transform=None, colorImageCv=None, threshold=0.1, ROI=None):
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
        points, colors = self.generatePointsRaw(depthImageCv, transform, colorImageCv, threshold, ROI)
        if len(colors) == 0:
            colors = None
        return mkshapes.makePoints(points, colors=colors, **kwargs)

def cameraModelFromCameraInfo(camera_info):
    pcm = PinholeCameraModel()
    pcm.from_camera_info(camera_info)
    return pcm
