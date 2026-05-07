# ---
# jupyter:
#   jupytext:
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.3'
#       jupytext_version: 1.18.1
#   kernelspec:
#     display_name: Choreonoid
#     language: python
#     name: choreonoid
# ---

# %%
exec(open('/choreonoid_ws/install/share/irsl_choreonoid/sample/irsl_import.py').read())

# %%
import rospy
from sensor_msgs.msg import Joy, CompressedImage
#
from threading import Lock
import time
#
import cv2
#
import cnoid.JupyterPlugin

# %%
## for jupyter
# from IPython.display import display
# import ipywidgets as widgets
# out = widgets.Output()
# display(out)
# out.append_stdout('out: \n')

# %%
class JoyParser(object):
    """
    Base Class
    """
    ##
    BTN_0 = 0
    BTN_1 = 1
    BTN_2 = 2
    BTN_3 = 3
    BTN_4 = 4
    TGL_0 = 5
    TGL_1 = 6
    TGL_2 = 7
    ##
    AXIS0_X = 0
    AXIS0_Y = 1
    AXIS0_Z = 2
    AXIS1_X = 3
    AXIS1_Y = 4
    AXIS1_Z = 5
    def __init__(self, **kwargs):
        self.lock = Lock()
        self.initializeROS(**kwargs)
        self.initialSettings()
        self.prev_buttons = None
        self.prev_axes = None
        self.prev_tm = None
        self.msg = None
    ##
    def initializeROS(self, **kwargs):
        try:
            rospy.init_node('testjoy', **kwargs)
        except Exception as e:
            pass
        self.sub = rospy.Subscriber('/webjoy', Joy,
                                    callback=self.callback_joy, queue_size=1)
    ##
    def initialSettings(self):
        #Abstruct
        pass
    ##
    def callback_motion(self):
        # rospy.loginfo(f'cb: {self}')
        #Abstruct
        pass
    ##
    def parseButton(self):
        #Abstruct
        return False
    def resetButtons(self):
        self.msg.buttons = [0]*len(self.prev_buttons)
    ##
    def callback_joy(self, msg):
        with self.lock:
            if self.prev_tm is not None:
                self.edge =  [ c - p for p, c in zip(self.prev_buttons, msg.buttons) ]
                self.msg = msg
                self.callback_motion()
            ## ignore the first msg
            self.prev_tm = msg.header.stamp
            self.prev_buttons = msg.buttons
            self.prev_axes = msg.axes
# %%
class JoyCoords(JoyParser):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.JoyDisplay = True
        self.trans_idx = [JoyParser.AXIS0_X, JoyParser.AXIS0_Y, JoyParser.AXIS0_Z]
        self.trans_sign = npa([1, 1, 1])
        self.rot_idx   = [JoyParser.AXIS1_X, JoyParser.AXIS1_Y, JoyParser.AXIS1_Z]
        self.rot_sign = npa([1, 1, 1])
    def makeTranslation(self, val_lst):
        return self.trans_sign * npa( [ val_lst[i] if i >= 0 else 0.0 for i in self.trans_idx ] )
    def makeRotation(self, val_lst):
        return self.rot_sign   * npa( [ val_lst[i] if i >= 0 else 0.0 for i in self.rot_idx ] )
    ##
    def callback_motion_diff(self):
        ## rospy.loginfo(f'cb: JoyCoordsDiff: {self}')
        diff = [ c - p for  p, c in zip(self.prev_axes, self.msg.axes) ]
        self.trans = self.makeTranslation(diff)
        self.rpy   = self.makeRotation(diff)
        ## rospy.loginfo(f'sub: {self.trans} {self.rpy}')
    ##
    def callback_motion(self):
        ## rospy.loginfo(f'cb: JoyCoords: {self}')
        diff = self.msg.axes
        self.trans = self.makeTranslation(diff)
        self.rpy   = self.makeRotation(diff)
        ## rospy.loginfo(f'sub: {self.trans} {self.rpy}')
# %%
class JoyDisplayCoords(JoyCoords):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.di = DrawInterface()
        self.di_cds = mkshapes.make3DAxis(radius=0.005, length=0.06, axisRatio=0.2)
        self.di.addObject(self.di_cds)
        #self.scale_trans = 0.04
        #self.scale_rot = 0.1
        self.scale_trans = 0.007
        self.scale_rot   = 0.021
        self.__enable = True
    ##
    def callback_motion(self):
        ## rospy.loginfo(f'cb: JoyDisplayCoords: {self}')
        if self.parseButton():
            return
        super().callback_motion()
        if self.__enable:
            ## rospy.loginfo(f'main: {self.trans} {self.rpy}')
            cds = coordinates(self.trans*self.scale_trans, self.rpy*self.scale_rot)
            self.di_cds.transform(cds)
    ##
    def parseButton(self):
        """
        """
        if self.edge[JoyParser.BTN_0] > 0:
            self.di_cds.newcoords(coordinates())
            return True ## exit
        return False
    ##
    def enableCoords(self, on=True):
        self.__enable = on
# %%
class JoyCameraCoords(JoyDisplayCoords):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.scale_cam_trans = 0.04
        self.scale_cam_rot = 0.1
        self.init_coords, self.init_fov = ib.getCameraCoords()
        self.cam_coords = self.init_coords.copy()
        self.__enable = True
        self.enableCoords(False)##
        ## TODO: eye -> eye_reset(draw eye), all_reset, z_length
    ##
    def callback_motion(self):
        ## rospy.loginfo(f'cb: JoyDisplayCoords: {self}')
        super().callback_motion()
        if self.__enable:
            ## rospy.loginfo(f'main: {self.trans} {self.rpy}')
            ## TODO: move eye and rotate around eye
            cds = coordinates(self.trans*self.scale_cam_trans, self.rpy*self.scale_cam_rot)
            self.cam_coords.transform(cds)
            self.updateCamera()
    ##
    def parseButton(self):
        """
        """
        if self.msg.buttons[JoyParser.TGL_1] == 0:
            self.enableCoords(True)
            self.enableCamera(False)
        else:
            self.enableCoords(False)
            self.enableCamera(True)
        #
        if self.edge[JoyParser.BTN_0] > 0:
            self.di_cds.newcoords(coordinates())
            return True ## exit
        #
        if self.edge[JoyParser.BTN_1] > 0:
            self.cam_coords.newcoords(self.init_coords)
            self.updateCamera()
            return True ## exit
        return False
    ##
    def updateCamera(self):
        ib.setCameraCoords(self.cam_coords, self.init_fov)
    ##
    def enableCamera(self, on=True):
        self.__enable = on
# %%
#class Main(JoyDisplayCoords):
class Main(object):
    def __init__(self, joyClass=JoyCameraCoords, height=480, **kwargs):
        self.joy = joyClass(**kwargs)
        self.pub = rospy.Publisher('/image_joy_camera0/image_raw/compressed', CompressedImage, queue_size=1)
        self.jbar = cnoid.JupyterPlugin.JupyterBar.instance()
        self.height = height
        try:
            self.jbar.getUserButton('Main')
        except Exception as e:
            self.jbar.addUserButton('Main')
    ##
    def main(self, rate=10):
        cntr = self.jbar.getUserButton('Main')
        while cntr == self.jbar.getUserButton('Main'):
            IU.processEvent()
            self.process()
            time.sleep(1/rate)
    ##
    def process(self):
        img = self.joy.di.getImage(True)
        sh = img.shape
        ## rospy.loginfo(f'sz: {(sh, (480, int(480*sh[1]/sh[0])))}')
        rs_img = cv2.resize(img, (int(self.height*sh[1]/sh[0]), self.height))
        ret, buf = cv2.imencode('.jpeg', rs_img)
        if ret:
            msg = CompressedImage()
            msg.format = 'jpeg'
            msg.data = buf.tobytes()
            self.pub.publish(msg)
