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
## TODO: minimum set of import
exec(open('/choreonoid_ws/install/share/irsl_choreonoid/sample/irsl_import.py').read())

# %%
import rospy
from sensor_msgs.msg import Joy
#
from threading import Lock
import time
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
    def initializeROS(self, joy_topic_name='/webjoy', **kwargs):
        try:
            rospy.init_node('testjoy', **kwargs)
        except Exception as e:
            pass
        self.sub = rospy.Subscriber(joy_topic_name, Joy,
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
        self.trans_mask = npa([1, 1, 1])
        self.rot_idx   = [JoyParser.AXIS1_X, JoyParser.AXIS1_Y, JoyParser.AXIS1_Z]
        self.rot_sign = npa([1, 1, 1])
        self.rot_mask = npa([1, 1, 1])
    def makeTranslation(self, val_lst):
        return self.trans_mask * self.trans_sign * npa( [ val_lst[i] if i >= 0 else 0.0 for i in self.trans_idx ] )
    def makeRotation(self, val_lst):
        return self.rot_mask   * self.rot_sign   * npa( [ val_lst[i] if i >= 0 else 0.0 for i in self.rot_idx ] )
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
        self.init_cam_coords, self.init_fov = ib.getCameraCoords()
        self.cam_coords = self.init_cam_coords.copy()
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
            self.cam_coords.newcoords(self.init_cam_coords)
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
class RobotControl(JoyCameraCoords):
    def __init__(self, robot_interface,
                 trans_index=None, rot_index=None, rot_sign=None, trans_sign=None, rot_mask=None, trans_mask=None,
                 endEffector=None, robotModel=None, resetAngles=None, move_step=0.1,
                 graspFunction=None, ungraspFunction=None,
                 **kwargs):
        """
        """
        ##
        if type(robot_interface) is str:
            self.ri = RobotInterface(robot_interface)
        else:
            self.ri = robot_interface
        ##
        self.robot = self.ri.getRobotModel() if robotModel is None else robotModel
        ##
        super().__init__(**kwargs)
        ##
        self.trans_idx  = [JoyParser.AXIS0_Z, JoyParser.AXIS0_Y, JoyParser.AXIS0_X] if trans_index is None else trans_index
        self.rot_idx    = [JoyParser.AXIS1_Z, JoyParser.AXIS1_X, JoyParser.AXIS1_Y] if rot_index   is None else rot_index
        self.trans_sign = fv(1, 1, 1)  if trans_sign is None else trans_sign
        self.rot_sign   = fv(0, 1, -1) if rot_sign   is None else rot_sign
        self.trans_mask = fv(1, 1, 1) if trans_mask is None else trans_mask
        self.rot_mask   = fv(1, 1, 1) if rot_mask   is None else rot_mask
        ##
        if callable(endEffector):
            res = endEffector(robot)
            self.robotEndEffector = self.robot.getLimb(res)
        elif type(endEffector) is str:
            self.robotEndEffector = self.robot.getLimb(endEffector)
        else:
            raise Exception(f'endEffector: {endEffector} should be callable or str')

        if resetAngles is None:
            robot.setModelInitialPose()
            self.resetAngles = robot.angleVector()
        else:
            self.resetAngles = resetAngles
            robot.angleVector(resetAngles)

        self.move_step = move_step
        self.di_cds.newcoords(self.robotEndEffector.endEffector)

        self.graspFunction = graspFunction
        self.unGraspFunction = unGraspFunction

    def setInitialAngle(self, tm=4.0, *args, **kwargs):
        av = self.robot.angleVector(self.resetAngles)
        self.sendAngles(tm=tm)
        self.ungrasp(*args, **kwargs)
        self.hold = False

    def setAxisToEndEffector(self):
        self.di_cds.newcoords(self.robotEndEffector.endEffector)

    def callback_motion(self):
        super().callback_motion()
        ##
        ret, loop = self.solveIK(coordinates(self.di_cds.pos, self.di_cds.rot))
        ##
        if self.use_yaw:
            yaw = self.msg.axes[JoyParser.AXIS1_Z] * 0.04
            if yaw != 0.0:
                ang = self.robot.jointAngle('LINK_0')
                self.robot.setAngleMap({'LINK_0': ang - yaw})
        ##
        if ret:
            self.sendAngles(tm=self.move_step)
        self.setAxisToEndEffector()
    ##
    def parseButton(self):
        """
        """
        #if self.msg.buttons[JoyParser.TGL_0] == 0: ## move world
        #    pass
        #else:
        #    pass
        if self.msg.buttons[JoyParser.TGL_1] == 0: ## toggle camera-mode <-> coords-mode
            self.enableCoords(True)
            self.enableCamera(False)
        else:
            self.enableCoords(False)
            self.enableCamera(True)
        if self.msg.buttons[JoyParser.TGL_2] == 0: ## use yaw
            self.use_yaw = True
            self.rot_sign   = fv(0, 1, -1)
        else:
            self.use_yaw = False
            self.rot_sign   = fv(1, 1, -1)
        #
        if self.edge[JoyParser.BTN_0] > 0: ## reset coords-position
            self.di_cds.newcoords(self.robotEndEffector.endEffector)
            self.resetButtons()
            return True ## exit
        #
        if self.edge[JoyParser.BTN_1] > 0: ## reset camera-coords
            self.cam_coords.newcoords(self.init_cam_coords)
            self.updateCamera()
            self.resetButtons()
            return True ## exit
        #
        if self.edge[JoyParser.BTN_3] > 0: ## reset robot-posture
            rospy.loginfo('start-reset')
            self.setInitialAngle()
            self.di_cds.newcoords(self.robotEndEffector.endEffector)
            self.ri.waitUntilFinish(timeout=4.5)
            rospy.loginfo('end-reset')
            self.resetButtons()
            return True ## exit
        #
        if self.edge[JoyParser.BTN_4] > 0: ## toggle grasp<->ungrasp
            if self.hold:
                self.hold = False
                rospy.loginfo('start-ungrasp')
                self.ungrasp()
                self.ri.waitUntilFinish(timeout=1.5, group='gripper')
                rospy.loginfo('end-ungrasp')
            else:
                self.hold = True
                rospy.loginfo('start-grasp')
                self.grasp()
                rospy.loginfo('end-grasp')
            self.resetButtons()
            return True ## exit
        return False
    ###
    def solveIK(self, cds):
        return self.robot.arm.inverseKinematics(cds)
    ###
    def sendAngles(self, tm = 0.02):
        self.ri.sendAngleVector(self.robot.angleVector(), tm = tm)
    ##
    def grasp(self, *args, **kwargs):
        if callable(self.graspFunction):
            return self.graspFunction(self.ri, self.robot, *args, **kwargs)
    ##
    def ungrasp(self, *args, **kwargs):
        if callable(self.unGraspFunction):
            return self.unGraspFunction(self.ri, self.robot, *args, **kwargs)
#>    ####
#>    def grasp_implement(self, dt = 0.01, grasptime = 1.0, maxeffort = 80, graspaxis = 6):
#>        self.robot.setAngleMap({self.HAND_LINK: self.close_angle})
#>        self.ri.sendAngleVector(self.robot.angleVector(), grasptime, 'gripper')
#>        ##
#>        for t in range(int(grasptime//dt)):
#>            if self.ri.effortVector[graspaxis] > maxeffort:
#>                self.ri.sendAngleVector(self.ri.actualAngleVector, 0.05, 'gripper')
#>                break
#>            time.sleep(dt)
#>    def ungrasp_implement(self, grasptime = 1.0):
#>        self.robot.setAngleMap({self.HAND_LINK: self.open_angle})
#>        self.ri.sendAngleVector(self.robot.angleVector(), grasptime, 'gripper')

# %%
#>class Main(object):
#>    def __init__(self, joyClass=JoyCameraCoords, height=480, **kwargs):
#>        self.joy = joyClass(**kwargs)
#>        self.height = height
#>        self.pub = rospy.Publisher('/image_joy_camera0/image_raw/compressed', CompressedImage, queue_size=1)
#>        self.jbar = cnoid.JupyterPlugin.JupyterBar.instance()
#>        try:
#>            self.jbar.getUserButton('Main')
#>        except Exception as e:
#>            self.jbar.addUserButton('Main')
#>    ##
#>    def main(self, rate=10):
#>        cntr = self.jbar.getUserButton('Main')
#>        while cntr == self.jbar.getUserButton('Main'):
#>            IU.processEvent()
#>            self.process()
#>            time.sleep(1/rate)
#>    ##
#>    def process(self):
#>        img = self.joy.di.getImage(True)
#>        sh = img.shape
#>        ## rospy.loginfo(f'sz: {(sh, (480, int(480*sh[1]/sh[0])))}')
#>        rs_img = cv2.resize(img, (int(self.height*sh[1]/sh[0]), self.height))
#>        ret, buf = cv2.imencode('.jpeg', rs_img)
#>        if ret:
#>            msg = CompressedImage()
#>            msg.format = 'jpeg'
#>            msg.data = buf.tobytes()
#>            self.pub.publish(msg)
