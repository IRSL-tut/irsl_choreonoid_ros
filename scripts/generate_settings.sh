#!/bin/bash

set -e

# See http://unix.stackexchange.com/questions/101080/realpath-command-not-found
realpath ()
{
    f=$@;
    if [ -d "$f" ]; then
        base="";
        dir="$f";
    else
        base="/$(basename "$f")";
        dir=$(dirname "$f");
    fi;
    dir=$(cd "$dir" && /bin/pwd);
    echo "$dir$base"
}


BODYFILE=''
RICONFIGFILE=robot_interface.yaml
WORLDSETTINGFILE=world.yaml


USE_WHEEL=False
USE_ARM=True
WHEEL_LIST=()
DEFAULT_CONTROLLER=trajectory_controller
CONTROLLERS=("${DEFAULT_CONTROLLER}" "joint_state_controller")
CONTROLLERS_STR="${CONTROLLERS[*]}"
GEN_TYPE=2 ## generate world.yaml 1

while [[ $# -gt 0 ]]; do
    case $1 in
        -b|--body)
            BODYFILE="$2"
            shift
            shift
            ;;
        --help)
            echo "generator_settings.sh [ -b | --body <bodyfile.body> ]"
            exit 0
            ;;
        --)
            shift
            break
            ;;
        *)
            POSITIONAL_ARGS+=("$1") # save positional arg
            shift # past argument
            ;;
    esac
done

if [ -z "$BODYFILE" ]; then
    echo "Please input body file using -b/--body <bodyfile>"
    exit
fi

if [ ! -e "$BODYFILE" ]; then
    echo "BODYFILE ${BODYFILE} does not exist!"
    exit
fi

set -x

ROBOTNAME=`python3 -c "import cnoid.Body;import sys;print(cnoid.Body.BodyLoader().load('$BODYFILE').getModelName())"`

URDFFILE=`echo $BODYFILE |sed 's/.body$/.urdf/g'`

choreonoid_body2urdf $BODYFILE > $URDFFILE 2>/dev/null

# generate RI settings
rosrun irsl_choreonoid_ros generate_ri_config.py --bodyfile $BODYFILE --controller_name ${DEFAULT_CONTROLLER} --use_wheel $USE_WHEEL   > $RICONFIGFILE

# generate simulate setting
rosrun irsl_choreonoid_ros generate_roslaunch.py --bodyfile $BODYFILE --controllers "${CONTROLLERS_STR}" --demo_base_dir `pwd` --urdffile $URDFFILE --worldsettings $WORLDSETTINGFILE > run_sim_robot.launch
rosrun irsl_choreonoid_ros generate_world_config.py  --bodyfile $BODYFILE --joint_controller_name ${DEFAULT_CONTROLLER} > $WORLDSETTINGFILE

# generate real robot setting files
rosrun irsl_choreonoid_ros generate_robot_sensor_config.py --bodyfile $BODYFILE > sensor_config.yaml
rosrun irsl_choreonoid_ros generate_dxl_shm_config.py --bodyfile $BODYFILE > dynamixel_config.yaml
rosrun irsl_choreonoid_ros generate_ros_control.py    --bodyfile $BODYFILE --controller_name ${DEFAULT_CONTROLLER} > ros_control.yaml
rosrun irsl_choreonoid_ros generate_jointlist.py      --bodyfile $BODYFILE > jointlist.yaml
