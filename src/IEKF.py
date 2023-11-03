import numpy as np
from States import RobotState
from NoiseParam import NoiseParam

class InEKF:
    def __init__(self, state, params):
        self.g_ = np.array([0, 0, -9.81])
        self.state_ = state
        self.noise_params_ = params
    
    
    # phase 1: project ahead
    # hat(X_{t+1}) = Phi@hat(X_{t})
    # P^{-}_{k+1} = Phi_{k}@P_{k}
    def propogate(self, m, dt):
        w = m[:3]-self.state_.getGyroscopeBias()
        a = m[3:]-self.state_.getAccelerometerBias() 

        X = self.state_.getX() # (5+num_C,5+num_C) 
        P = self.state_.getP_() # (15+num_C,15+num_C) TODO: update this comment

        R = self.state_.getRotation() #(3,3)
        v = self.state_.getVelocity() #(3,1)
        p = self.state_.getPosition() #(3,1)

        phi = w*dt
        print(phi)


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

def testInEKFPropogate():
    state = RobotState()
    param = NoiseParam()
    inefk = InEKF(state=state, params=param)
    inefk.propogate(np.random.randn(6,1), dt=0.1)

if __name__=="__main__":
    #testInEFKInit()
    testInEKFPropogate()