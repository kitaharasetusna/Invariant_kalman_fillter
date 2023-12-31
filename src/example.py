from IEKF import InEKF
from states import RobotState
from NoiseParam import NoiseParam
import numpy as np
from visualization import VisulizeSE3Anim
from scipy.spatial.transform import Rotation as Rsp_
from IEKF import Kinematics

DT_MIN = 1e-6
DT_MAX = 1

# 1. SOS initialization for the filter(state and noise)
initial_state = RobotState()
R0 = np.eye(3)
R0[1,1] = -1; R0[2,2] = -1
# print(R0)
v0 = np.zeros((3,1))
p0 = np.zeros((3,1))
bg0 = np.zeros((3,1))
ba0 = np.zeros((3,1))
initial_state.setRotation(R=R0)
initial_state.setVelocity(V=v0)
initial_state.setPosition(P=p0)
initial_state.setGyroscopeBias(bg=bg0)
initial_state.setAccelerometerBias(ba=ba0)

noiseparam = NoiseParam()

bool_noise = True
if bool_noise == True:
    noiseparam.setGyroscopeNoise(0.01)
    noiseparam.setAccelerometerNoise(0.1)
    noiseparam.setGyroscopeBiasNoise(0.00001)
    noiseparam.setAccelerometerBiasNoise(0.0001)
    noiseparam.setContactNoise(0.01)

inekf = InEKF(state=initial_state, params=noiseparam)
inekf.debug_print()
# 1. EOS initialization for the filter(state and noise)

imu_measurement = np.zeros((6, 1))
imu_measurement_prev = np.zeros((6, 1))
t = 0.0
t_prev = 0.0

file_path = './data/imu_kinematic_measurements.txt'
se3_matrices = []
with open(file_path, 'r') as file:
    # Iterate over each line in the file
    cnt = 0
    for line in file:
        # if cnt==5:
        #     break
        # Process the line as needed
        # print(line.strip())  # This removes leading and trailing whitespaces (including '\n')
        cur_line = line.strip()
        measurement = cur_line.split()
        if measurement[0]=="IMU":
            print("getting IMU events")
            assert len(measurement)-2==6, "imu data is wrong, it should have 6 data"
            t = float(measurement[1])
            imu_measurement[:] = np.array([float(imu_data) for imu_data in measurement[2:]]).reshape(6,1)
            # print(imu_measurement.shape, type(imu_measurement), type(imu_measurement[0]))
            dt = t - t_prev
            if dt > DT_MIN and dt < DT_MAX:
                print('propogating...')
                inekf.propogate(m=imu_measurement, dt=dt)

        elif measurement[0]=="CONTACT":
            print("getting contact events")
            assert (len(measurement)-2)%2==0, "contact data is wrong, it should have 2*num_contact_sensors"
            contacts = []
            t = float(measurement[1])
            for i in range(2, len(measurement), 2):
                id_ = int(float(measurement[i]))
                # print(id_); import sys; sys.exit()
                indicator = bool(float(measurement[i+1])) 
                contacts.append((id_, indicator))
            inekf.setContacts(contacts=contacts)

        elif measurement[0]=="KINEMATIC":
            print("getting kinematic events")
            assert (len(measurement)-2)%44==0, "kinetic data is wrong, it should have 44*num_kinetic_sensors_data"
            pose = np.eye(4)
            covariance = np.zeros((6, 6))
            measured_kinematics = []
            t = float(measurement[1])
            for i in range(2, len(measurement), 44):
                # TODO: normalize
                R_ = Rsp_.from_quat([float(measurement[i+1]), float(measurement[i+2]),\
                    float(measurement[i+3]), float(measurement[i+4])])
                R_ = np.array(R)
                # print(R_, type(R_), R_.shape); import sys; sys.exit()
                p = np.array([measurement[i+5], measurement[i+6], measurement[i+7]]) 
                pose[:3, :3] = R_
                pose[:3, 3] = p.reshape(3,)
                # print(pose, pose.shape); import sys; sys.exit()
                for j in range(6):
                    for k in range(6):
                        covariance[j, k] = float(measurement[i+8+ j*6+k])
                frame = Kinematics(id=id, pose=pose, covariance=covariance) 
                measured_kinematics.append(frame)
            inekf.CorrectKinematics(measured_kinematics=measured_kinematics)

        else:
            raise ValueError(measurement[0]) 
        
        t_prev = t 
        imu_measurement_prev = imu_measurement

        
        
        if cnt%100==0:
            R = inekf.state_.getRotation()
            p = inekf.state_.getPosition()
            tmp_mat = np.eye(4)
            tmp_mat[:3,:3] = R; tmp_mat[:3, 3] = p.reshape(3, )
            se3_matrices.append(100*tmp_mat)
            print(tmp_mat.shape)
        # for debugging
        cnt +=1

print(len(se3_matrices), se3_matrices[0], se3_matrices[100], se3_matrices[200], se3_matrices[-1])
VisulizeSE3Anim(se3_matrices)