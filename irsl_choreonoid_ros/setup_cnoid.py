### for development version
from cnoid.Base import RootItem
from cnoid.Base import ItemTreeView

import irsl_choreonoid.cnoid_util as iu

import yaml
import math
import sys

## ROS
from cnoid.ROSPlugin import WorldROSItem
from cnoid.ROSPlugin import BodyROSItem
from cnoid.ROSPlugin import ROSControlItem
#from irsl_choreonoid_ros.cnoid_ros_util import parseURLROS
from .cnoid_ros_util import parseURLROS
import rosgraph
import rospy

from irsl_choreonoid.setup_cnoid import _applyParameter
from irsl_choreonoid.setup_cnoid import _getDictValue
from irsl_choreonoid.setup_cnoid import _getDictValueExist

from irsl_choreonoid.setup_cnoid import SetupCnoid as SetupCnoidOrg
from irsl_choreonoid.setup_cnoid import _BodyItemWrapper as _BodyItemWrapperOrg

#### specification of yaml
## robot:
##   model: @robot_file_name@
##   name: MyRobot
##   initial_joint_angles: []
##   initial_coords: {} ## using make_coordinates from irsl_choreonoid.robot_util
##   BodyROSItem: ## should be launch from choreonoid_ros
##       joint_state_publication: false
##       joint_state_update_rate: 100
##       name_space: arm_robot3
##   ROSControlItem: ## should be launch from choreonoid_ros
##       name_space: arm_robot3
## robots:
##   - { robot: }
## object:
##   model: @object_file_name@
##   name: MyObject
##   initial_joint_angles: []
##   initial_coords: {} ## using make_coordinates from irsl_choreonoid.robot_util
##   fixed: True ##
## objects:
##   - { object: }
## world:
##   World:
##     name:
##     draw_grid: False
##   Simulator:
##     type: 'AISTSimulator'
##   GLVision:
##   Camera:
##     lookEye:
##     lookForDirection:
##     lookAtCenter:
##     lookAtUp:
##     position: { pos: [], aa: [] }
##     fov:
##   WorldROS: ## should be launch from choreonoid_ros
##   ROS:
##     urdf_settings:
##          file: @file_name@
##          robotName:
##          transmission:
##          name:
##     set_parameter: 
##          - type: 'yaml'
##            file: @file_name@
##            name: 
##          - type: 'param'
##            name: 
##            parameter: {}
##          - parameter: {}
##     generate_settings:
##        robot: @robot_file_name@
##        controllers: [ {name: '', type: '', joints: [] } ]
## pythonScript:
####

class _BodyItemWrapperROS(_BodyItemWrapperOrg):
    ##
    def addRobot(self, info, fix=False, ros_enable=False):
        super().addRobot(info, fix=fix)
        check = True
        if 'no_check' in info and info['no_check']:
            check = False
        if ros_enable:
            if 'BodyROSItem' in info:
                self.addBodyROSItem(self.body_item, check=check, param=info['BodyROSItem'])
            if 'ROSControlItem' in info:
                self.addROSControlItem(self.body_item, check=check, param=info['ROSControlItem'])

    def addBodyROSItem(self, parent, check=True, param=None):
        self.body_ros = BodyROSItem()
        self.body_ros.setName('BodyROSItem')
        # body_ros_item.nameSpace =
        # body_ros_item.jointStateUpdateRate =
        # body_ros_item.publishJointState =
        if parent is not None:
            parent.addChildItem(self.body_ros)
        if param is not None:
            _applyParameter(self.body_ros, param)
        if check:
            ItemTreeView.instance.checkItem(self.body_ros)

    def addROSControlItem(self, parent, check=True, param=None):
        self.ros_control = ROSControlItem()
        self.ros_control.setName('ROSControlItem')
        ## ros_cntrl_item.nameSpace = ''
        if parent is not None:
            parent.addChildItem(self.ros_control)
        if param is not None:
            _applyParameter(self.ros_control, param)
        if check:
            ItemTreeView.instance.checkItem(self.ros_control)

class SetupCnoid(SetupCnoidOrg):
    def __init__(self, rootItem=None, worldItem=None):
        super().__init__(rootItem=rootItem, worldItem=worldItem)
        self.ros_enable = False
        self._parseURL = parseURLROS

    def _getWrapper(self, worldItem=None, offset=None):
        if worldItem is not None:
            bi = _BodyItemWrapperROS(worldItem, parseURL=self._parseURL, offset=offset)
        else:
            bi = _BodyItemWrapperROS(self.world_item, parseURL=self._parseURL, offset=offset)
        return bi

    def _addWorldROS(self, name='WorldROS', check=True, param=None):
        ros_world = self.root_item.findItem(name)
        if ros_world is None:
            self.ros_world = WorldROSItem()
            self.ros_world.setName(name)
            _applyParameter(self.ros_world, param)
            self.ros_world.maxClockPublishingRate = 100
            self.world_item.addChildItem(self.ros_world)## ? insert
        else:
            self.ros_world = ros_world
        #
        if check:
            ItemTreeView.instance.checkItem(self.ros_world)

    def addExtraWorld(self, world_info):
        ## ROS
        is_master_exists = rosgraph.is_master_online()
        exist_, ros_ = _getDictValueExist(world_info, ('ros', 'ROS', 'Ros'))
        if exist_:
            if is_master_exists:
                self.ros_enable = True
                self._execROSScript(param=ros_)
        ## WorldROS
        exist_, world_ros_ = _getDictValueExist(world_info, ('WorldROS', 'world_ros'))
        if exist_:
            if is_master_exists:
                self.ros_enable = True
                self._addWorldROS(param=world_ros_)
        elif self.ros_enable:
            if is_master_exists:
                self._addWorldROS()

    def _execROSScript(self, param=None):
        if param is None:
            return
        urdf_ = _getDictValue(param, ('urdf_settings', 'URDFSettings', 'URDF_settings', 'urdf_setting', 'URDFSetting', 'URDF_setting', 'URDF', 'urdf'))
        if urdf_ is not None:
            self._parseURDF(urdf_)
        parameters_ = _getDictValue(param, ('set_parameter', 'set_parameters', 'setParameter', 'setParameters', 'parameters', 'Parameters'))
        if parameters_ is not None:
            self._parseROSParam(parameters_)
        gen_set_ = _getDictValue(param, ('generate_settings', 'generateSettings', 'generate'))
        if gen_set_ is not None:
            self._generateROSParam(gen_set_)

    def _parseSingleParam(self, param):
        if 'type' in param:
            if param['type'] == 'yaml':
                file_ = _getDictValue(param, ('file', 'File', 'URI', 'uri', 'yaml', 'yaml_file'))
                if file_ is not None:
                    fn = self._parseURL(file_)
                    with open(fn) as f:
                        yparam = yaml.safe_load(f)
                    name_ = _getDictValue(param, ('name', 'param', 'Name', 'Param', 'parameter_name', 'parameterName'))
                    if name_ is not None:
                        rospy.set_param(name_, yparam)
                    else:
                        for k,v in yparam.items():
                            if type(k) is str:
                                rospy.set_param(k, v)
                return
        if 'parameter' in param:
            p = param['parameter']
            name_ = _getDictValue(param, ('name', 'param', 'Name', 'Param', 'parameter_name', 'parameterName'))
            if name_ is not None:
                rospy.set_param(name_, p)
            else:
                for k,v in p.items():
                    if type(k) is str:
                        rospy.set_param(k, v)

    def _parseROSParam(self, param):
        if type(param) is not list and type(param) is not tuple:
            ####
            return
        for one in param:
            if type(one) is dict:
                self._parseSingleParam(one)

    def _generateROSParam(self, param):
        model_ = _getDictValue(param, ('robot', 'model', 'Model', 'file', 'File', 'body', 'Body', 'model_file', 'modelFile'))
        if model_ is None:
            return
        nspace_ = _getDictValue(param, ('ns', 'name_space', 'nameSpace', 'robotName', 'name'))

        fname = self._parseURL(model_)
        robot_ = iu.loadRobot(fname)

        if nspace_ is None:
            if len(model_.name) > 0:
                nspace_ = robot_.name
            else:
                nspace_ = robot_.modelName

        if type(param) is dict:
            cont_list_ = _getDictValue(param, ('controllers', 'Controllers'))
        else:
            cont_list_ = param

        urdf_str = _generate_joint_urdf_header(name=nspace_) ## TODO param
        cont_param = _generate_roscontrol_config_base()## TODO param

        for cont_ in cont_list_:
            nm_   = _getDictValue(cont_, ('name', 'Name', 'controller_name', 'controllerName'))
            tp_   = _getDictValue(cont_, ('type', 'Type', 'controller_type', 'controllerType'))
            gp_   = _getDictValue(cont_, ('P', 'pgain', 'Pgain', 'PGain'), 200.0)
            gd_   = _getDictValue(cont_, ('D', 'dgain', 'Dgain', 'DGain'), 4.0)
            gi_   = _getDictValue(cont_, ('I', 'Igain', 'Igain', 'IGain'), 0.0)
            jnms_ = _getDictValue(cont_, ('joints', 'joint_list', 'joint_names', 'jointList', 'jointNames'))
            if nm_ is None:
                continue
            if jnms_ is None or ( type(jnms_) is str and jnms_.lower() == 'all' ):
                jlst_ = robot_.joints
            else:
                jlst_ = [ robot_.joint(j) for j in jnms_ ]
            if tp_ is not None:
                res = _generate_roscontrol_config(jlst_, gain_p=gp_, gain_d=gd_, gain_i=gi_, controller_type=tp_.lower())
                cont_param[nm_] = res
                urdf_str += _generate_joint_urdf_joint(jlst_, interface_type=tp_.capitalize())
            else:
                res = _generate_roscontrol_config(jlst_)
                cont_param[nm_] = res
                urdf_str += _generate_joint_urdf_joint(jlst_)
        urdf_str += _generate_joint_urdf_footer()

        rospy.set_param('{}'.format(nspace_), cont_param)
        rospy.set_param('{}/robot_description'.format(nspace_), urdf_str)

    def _addRobot(self, info=None, worldItem=None, offset=None):
        bi = self._getWrapper(worldItem=worldItem, offset=offset)
        bi.addRobot(info, ros_enable=self.ros_enable)

def _generate_roscontrol_config_base(joint_state_publish_rate=50):
    # controller_type: 'position', 'effort', 'velocity'
    param = {}
    param['joint_state_controller'] = {'type': 'joint_state_controller/JointStateController',
                                       'publish_rate': joint_state_publish_rate}
    return param

def _generate_roscontrol_config(jointList, gain_p=200.0, gain_i=0.0, gain_d=4.0, controller_type='position'):
    # controller_type: 'position', 'effort', 'velocity'
    controller_param = {'type': '{}_controllers/JointTrajectoryController'.format(controller_type) }
    names = [] # names
    gains_param = {}
    for j in jointList:
        if j is None:
            continue
        nm = j.jointName
        names.append( nm )
        gains_param[nm] = {'p': gain_p, 'd': gain_d, 'i': gain_i }
    controller_param['joints'] = names
    controller_param['gains'] = gains_param

    return controller_param

def _generate_joint_urdf_header(robot=None, name=None):
    if name is None:
        if robot is None:
            name = 'robot'
        else:
            if len(robot.name) > 0:
                name = robot.name
            else:
                name = robot.modelName
    res = '<?xml version="1.0" ?>\n<robot name="{}">\n'.format(name)
    res += '<link name="root" />\n'
    return res

def _generate_joint_urdf_footer():
    return '</robot>\n'

def _generate_joint_urdf_joint(jointList, interface_type='Position'):
    # controller_type: 'Position', 'Effort', 'Velocity'

    res = ''
    for j in jointList:
        if j is None:
            continue
        ##
        q_upper = j.q_upper
        q_lower = j.q_lower
        if j.q_upper >= float('inf'):
            q_upper = sys.float_info.max
        if j.q_lower <= float('-inf'):
            q_lower = -sys.float_info.max
        ##
        jname = j.jointName
        res += '<link name="link_{}" />\n'.format(jname)

        res += '<joint name="{}" type="revolute">\n'.format(jname)
        res += '  <parent link="root" />\n'
        res += '  <child  link="link_{}" />\n'.format(jname)
        res += '  <limit lower="{}" upper="{}" effort="{}" velocity="{}" />\n'.format(q_lower, q_upper,
                                                                                      j.u_upper if j.u_upper < math.fabs(j.u_lower) else math.fabs(j.u_lower),
                                                                                      j.dq_upper if j.dq_upper < math.fabs(j.dq_lower) else math.fabs(j.dq_lower))
        res += '</joint>\n'.format(jname)

        res += '<transmission name="{}_trans">\n'.format(jname)
        res += '  <type>transmission_interface/SimpleTransmission</type>\n'
        res += '  <joint name="{}">\n'.format(jname)
        res += '    <hardwareInterface>hardware_interface/{}JointInterface</hardwareInterface>\n'.format(interface_type)
        res += '  </joint>\n'
        res += '  <actuator name="{}_motor">\n'.format(jname)
        res += '    <mechanicalReduction>1</mechanicalReduction>\n'
        res += '  </actuator>\n'
        res += '</transmission>\n'

    return res
