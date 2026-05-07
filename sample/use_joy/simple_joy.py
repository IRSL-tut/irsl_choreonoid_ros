from joy import JoyParser, JoyCameraCoords
import rospy
import time

class RobotControl(JoyCameraCoords):
    def __init__(self, **kwargs):
        self.ri = RobotInterface('robot_interface.yaml')
        self.robot = self.ri.getRobotModel()
        ##
        super().__init__(**kwargs)
        #
        self.trans_idx = [JoyParser.AXIS0_Z, JoyParser.AXIS0_Y, JoyParser.AXIS0_X]
        #self.trans_sign = fv(1, 1, 1)
        self.rot_idx   = [JoyParser.AXIS1_Z, JoyParser.AXIS1_X, JoyParser.AXIS1_Y ]
        self.rot_sign   = fv(0, 1, -1)
        #
        self.robot.registerEndEffector('arm', ## end-effector
                                       'LINK_5', ## tip-link
                                       tip_link_to_eef=coordinates(fv(0, 0, 0.12), fv(1/math.sqrt(2), 0.0, 1/math.sqrt(2), 0.0)),
                                       joint_list = ['LINK_0', 'LINK_1', 'LINK_2', 'LINK_3', 'LINK_4', 'LINK_5']
                                       )
        self.resetAngles = fv(-0.00,  0.9,  0.6, 0,  0.1, 0, 0)
        self.open_angle = -0.2
        self.close_angle = 1.082
        self.HAND_LINK='LINK_6'
        self.move_step = 0.1
        self.di_cds.newcoords(self.robot.arm.endEffector)

    def setInitialAngle(self, tm=4.0):
        av = self.robot.angleVector(self.resetAngles)
        self.sendAngles(tm=tm)
        self.ungrasp(grasptime=tm)
        self.hold = False

    def setInitialAxis(self):
        self.di_cds.newcoords(self.robot.arm.endEffector)

    def callback_motion(self):
        super().callback_motion()
        ##
        ret, loop = self.solveIK(coordinates(self.di_cds.pos, self.di_cds.rot))
        ##
        if use_yaw:
            yaw = self.msg.axes[JoyParser.AXIS1_Z] * 0.04
            if yaw != 0.0:
                ang = self.robot.jointAngle('LINK_0')
                self.robot.setAngleMap({'LINK_0': ang - yaw})
        ##
        if ret:
            self.ri.sendAngleVector(self.robot.angleVector(), tm = self.move_step)
        self.setInitialAxis()
    ##
    def parseButton(self):
        """
        """
        #if self.msg.buttons[JoyParser.TGL_0] == 0: ## move world
        #    pass
        #else:
        #    pass
        if self.msg.buttons[JoyParser.TGL_1] == 0:
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
        if self.edge[JoyParser.BTN_0] > 0:
            self.di_cds.newcoords(self.robot.arm.endEffector)
            self.resetButtons()
            return True ## exit
        #
        if self.edge[JoyParser.BTN_1] > 0:
            self.cam_coords.newcoords(self.init_coords)
            self.updateCamera()
            self.resetButtons()
            return True ## exit
        #
        if self.edge[JoyParser.BTN_3] > 0:
            rospy.loginfo('start-reset')
            self.setInitialAngle()
            self.di_cds.newcoords(self.robot.arm.endEffector)
            self.ri.waitUntilFinish(timeout=4.5)
            rospy.loginfo('end-reset')
            self.resetButtons()
            return True ## exit
        #
        if self.edge[JoyParser.BTN_4] > 0:
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
    def grasp(self, dt = 0.01, grasptime = 1.0, maxeffort = 80, graspaxis = 6):
        self.robot.setAngleMap({self.HAND_LINK: self.close_angle})
        self.ri.sendAngleVector(self.robot.angleVector(), grasptime, 'gripper')
        ##
        for t in range(int(grasptime//dt)):
            if self.ri.effortVector[graspaxis] > maxeffort:
                self.ri.sendAngleVector(self.ri.actualAngleVector, 0.05, 'gripper')
                break
            time.sleep(dt)
    def ungrasp(self, grasptime = 1.0):
        self.robot.setAngleMap({self.HAND_LINK: self.open_angle})
        self.ri.sendAngleVector(self.robot.angleVector(), grasptime, 'gripper')

#exec(open('simple_joy.py').read())
#rc = RobotControl()
