SIM_DT=0.005
RENDER_DT=0.04#渲染频率太高会导致仿真失真

SENCE_TERRAIN='scene_terrain.xml'
SENCE_PLANE='scene.xml'

USE_TERRAIN=True

DEFAULT_POS=[ 0.0,-0.44,  0.05,
             -0.0, 0.44, -0.05,
              0.0,-0.44, -0.05,
             -0.0, 0.44, -0.05]

NOISE_QUAT=8.36e-2
NOISE_GYRO=1.94e-1
NOISE_ACC=5.88e-2


MOTOR_DELAY=0.003#s latency  13ms
ENCODER_POSITION_NOISE=0.000001
ENCODER_VELOCITY_NOISE=0.01

# MOTOR_DELAY=0.
# ENCODER_POSITION_NOISE=0.01
# ENCODER_VELOCITY_NOISE=0.01

# MOTOR_BIAS=0.05
MOTOR_BIAS=0.01

ACTUATOR_NUM=12

MOTOR_MAPPING=[3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]
#MOTOR_MAPPING=[0,1,2,3,4,5,6,7,8,9,10,11]
DEFAULT_POS=[ 0.0,-0.44,  0.05,
             -0.0, 0.44, -0.05,
              0.0,-0.44, -0.05,
             -0.0, 0.44, -0.05]
# ACTUATOR_NUM=6

# MOTOR_MAPPING=[0, 1, 2, 3, 4, 5]
# DEFAULT_POS=[ -1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0]

#to do:可以把joint mapping，joint/sensor index放在这里

