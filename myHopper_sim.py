import time
import numpy as np
import pybullet as p
import pybullet_data as pd


def contact():
    tip_z_position = p.getLinkState(hopperID, tipLinkIndex)[0][2]

    if tip_z_position < (0.01 + 0.002):
        return True
    else:
        return False


def getVelocity():
    base_vel = np.array(p.getBaseVelocity(hopperID)[0])
    link2_vel = np.array(p.getLinkState(hopperID, 0, 1)[6])

    hopper_vel = (base_vel + link2_vel) / 2

    return hopper_vel


def getTargetLegDisplacement():
    vel = getVelocity()[0:2]
    vel_error = vel-targetVelocity
    vel_gain = 0.027

    neutral_point = (vel*stance_duration)/2.0

    disp = neutral_point+vel_gain*vel_error
    disp = np.append(disp, -np.sqrt(getLegLength()**2-disp[0]**2-disp[1]**2))
    if np.isnan(disp[2]):
        print('legs too short')
    return disp

def getLegLength():
    return 0.15 - p.getJointState(hopperID,pneumatic_joint_index)[0]

def transform_H_to_B(vec): # transform a 4x1 vector in H frame to be expressed in B frame.
                            # H attaches to hip but stays parallel to World
                            # B is fixed with hip for both pos and orientation
    HB_matrix_row_form = p.getMatrixFromQuaternion(p.getBasePositionAndOrientation(hopperID)[1])
    HB_matrix = np.zeros((4,4))
    HB_matrix[3,3]=1
    HB_matrix[0, 0:3] = HB_matrix_row_form[0:3]
    HB_matrix[1, 0:3] = HB_matrix_row_form[3:6]
    HB_matrix[2, 0:3] = HB_matrix_row_form[6:9]

    HB_matrix = np.matrix(HB_matrix)

    BH_matrix = np.linalg.inv(HB_matrix)

    return BH_matrix*vec

p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

planeID = p.loadURDF("plane.urdf")
p.changeDynamics(planeID,-1,lateralFriction = 60)
p.resetDebugVisualizerCamera(cameraDistance=1.62, cameraYaw=47.6, cameraPitch=-30.8,
                             cameraTargetPosition=[0.43,1.49,-0.25])

# Place hopper in the air, and rotate it slightly so the hopper will fall over quickly if no control is done,
# instead of hopping in place many hops as if it's balancing, when it's only because the hopper is oriented almost "perfectly"
hopperID = p.loadURDF("hopper.urdf", [0, 0, 0.2], [0.00, 0.001, 0, 1])  

p.setJointMotorControl2(hopperID, 6, p.VELOCITY_CONTROL, force=0)

p.setGravity(0,0,-9.81)
curtime = 0
dt = 1. / 240.

k_flight = 1200  # leg spring constant during flight phase
k_stance = 2700  # leg spring constant during stance phase
state = 0  # 0 -> Stance, 1-> flight
legForce = 0
tipLinkIndex = 6

outer_hip_joint_index = 2  # positive joint angle represents positive roll (Assuming fixed leg)
inner_hip_joint_index = 5  # positive joint angle represents positive pitch (Assuming fixed leg)
pneumatic_joint_index = 6  # joint index of the springy leg

hip_joint_kp = 10
hip_joint_kd = 0.5

prev_orientation = np.array([0, 0, 0])
count = 0

stance_made = False
stance_duration = 0.17 #duration of a single stance phase. updated every stance. 0.17 is just rough value for the first hop.

targetVelocity = np.array([0.3,0.3])
#p.setRealTimeSimulation(1)
while 1:
    count = count + 1
    curtime = curtime + dt
    position = p.getJointState(hopperID, pneumatic_joint_index)[0]

    if contact():
        state = 0
    else:
        state = 1

    if state == 1: # Flight phase
        stance_made = False
        legForce = -(k_flight) * position
        targetLegDisplacement_H = getTargetLegDisplacement()
        targetLegDisplacement_H = np.append(targetLegDisplacement_H, 1)
        targetLegDisplacement_H = np.matrix(targetLegDisplacement_H)
        targetLegDisplacement_H = targetLegDisplacement_H.T

        targetLegDisplacement_B = transform_H_to_B(targetLegDisplacement_H)

        x_disp_B = targetLegDisplacement_B[0, 0]
        y_disp_B = targetLegDisplacement_B[1, 0]

        d = getLegLength()
        theta_inner = np.arcsin(x_disp_B/d)
        theta_outer = np.arcsin(y_disp_B/(-d*np.cos(theta_inner)))

        p.setJointMotorControl2(hopperID, outer_hip_joint_index, p.POSITION_CONTROL,
                                targetPosition=theta_outer)
        p.setJointMotorControl2(hopperID, inner_hip_joint_index, p.POSITION_CONTROL,
                                targetPosition=theta_inner)

    else:  # Stance phase
        if not stance_made:
            stance_made = True
            stance_duration = 0
        stance_duration = stance_duration + dt
        base_orientation = p.getLinkState(hopperID, 1)[1]
        base_orientation_euler = np.array(p.getEulerFromQuaternion(base_orientation))

        orientation_change = base_orientation_euler - prev_orientation
        orientation_velocity = orientation_change / dt

        outer_hip_joint_target_vel = -hip_joint_kp * base_orientation_euler[0] - hip_joint_kd * orientation_velocity[0]
        inner_hip_joint_target_vel = -hip_joint_kp * base_orientation_euler[1] - hip_joint_kd * orientation_velocity[1]

        p.setJointMotorControl2(hopperID, outer_hip_joint_index, p.VELOCITY_CONTROL,
                                targetVelocity=outer_hip_joint_target_vel)
        p.setJointMotorControl2(hopperID, inner_hip_joint_index, p.VELOCITY_CONTROL,
                                targetVelocity=inner_hip_joint_target_vel)

        prev_orientation = base_orientation_euler


        legForce = (-(k_stance) * position) - 20 #Add additional leg force to compensate for energy lost of the system

    p.setJointMotorControl2(hopperID, pneumatic_joint_index, p.TORQUE_CONTROL, force=legForce)
    
    #Print velocity of hopper every 100 time steps.
    if count % 100 == 0:
        print(getVelocity())


    #Changing target velocity of hopper over time
    if count == 1200:
        targetVelocity[0] = 0
        targetVelocity[1] = 0
    if count == 1680:
        targetVelocity[0] = -0.3
        targetVelocity[1] = 0
    if count == 2880:
        targetVelocity[0] = 0
        targetVelocity[1] = 0
    if count == 3360:
        targetVelocity[0] = 0.3
        targetVelocity[1] = -0.3
    if count == 4560:
        targetVelocity[0] = 0
        targetVelocity[1] = 0
    if count == 5040:
        targetVelocity[0] = -0.3
        targetVelocity[1] = 0
    if count == 6240:
        targetVelocity[0] = 0
        targetVelocity[1] = 0

    #Step simulation
    p.stepSimulation()
    time.sleep(dt)
