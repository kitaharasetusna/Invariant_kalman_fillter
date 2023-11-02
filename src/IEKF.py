import numpy as np
from States import RobotState
from NoiseParam import NoiseParam

class InEKF:
    def __init__(self, state, params):
        self.g_ = np.array([0, 0, -9.81])
        self.state_ = state
        self.noise_params_ = params


#----------------------------------------------for testing
def testInEFKInit():
    state = RobotState()
    param = NoiseParam()
    inefk = InEKF(state=state, params=param)
    print(inefk.state_.getX())
    print(inefk.state_.getP_().shape)
    print(inefk.state_.getTheta())
    print(inefk.noise_params_.getGyroscopeCov())
    print(inefk.noise_params_.getAccelerometerCov())
    print(inefk.noise_params_.getGyroscopeBiasCov())
    print(inefk.noise_params_.getAccelerometerBiasCov())
    print(inefk.noise_params_.getContactCov())

if __name__=="__main__":
    testInEFKInit()