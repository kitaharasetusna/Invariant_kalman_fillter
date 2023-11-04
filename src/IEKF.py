import numpy as np
from states import RobotState
from NoiseParam import NoiseParam
from matrixUtills import EXPSO3, skew, Adjoint_SEK3

class InEKF:
    def __init__(self, state, params):
        self.g_ = np.array([0, 0, -9.81]).reshape(-1,1)
        self.state_ = state
        self.noise_params_ = params
        self.estimated_contact_positions_ = []
    
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
        R_pred = R@EXPSO3(phi) # (3,3) rotation in IMU fram to rotation in world frame
        v_pred = v+(R@a+self.g_)*dt #(3,1)
        p_pred = p+v*dt+0.5*(R@a+self.g_)*dt*dt #(3,3)

        self.state_.setRotation(R_pred)        
        self.state_.setVelocity(v_pred)
        self.state_.setPosition(p_pred)

        # for debugging 
        # print(self.state_.getX())

        #-----linearize the invariant error dynamics A_t
        dimX = self.state_.dimX()
        dimP = self.state_.dimP()
        dimTheta = self.state_.dimTheta()

        # A in section IV.D
        A = np.zeros((dimP, dimP))
        A[3:6, :3] = skew(self.g_)
        A[6:9, 3:6] = np.eye(3)
        A[:3, dimP-dimTheta:dimP-dimTheta+3] = -R
        A[3:6, dimP-dimTheta+3:dimP-dimTheta+6] = -R

        for i in range(3, dimX):
            A[3*i-6:3*i-3, dimP-dimTheta:dimP-dimTheta+3] = -skew(X[0:3,i].reshape(-1,1))@R
            # print(-skew(X[0:3,i].reshape(-1,1))@R)

        # for i in range(len(A)):
        #     print(A[i])

        # covariance matrix w_t in section IV-D
        Qk = np.zeros((dimP, dimP))
        Qk[0:3, 0:3] = self.noise_params_.getGyroscopeCov()
        Qk[3:6, 3:6] = self.noise_params_.getAccelerometerCov()
        for it in self.estimated_contact_positions_:
            # TODO: update it when we got the estimated contact positions
            Qk[3+3*(it.second-3):3+3*(it.second-3)+3,3+3*(it.second-3):3+3*(it.second-3)+3] =\
                self.noise_params_.getContactCov()
        
        Qk[dimP-dimTheta:dimP-dimTheta+3 , dimP-dimTheta:dimP-dimTheta+3] = \
            self.noise_params_.getGyroscopeBiasCov()
        Qk[dimP-dimTheta+3:dimP-dimTheta+6, dimP-dimTheta+3:dimP-dimTheta+6] = \
            self.noise_params_.getAccelerometerBiasCov()

        # for i in range(len(Qk)):
        #     print(Qk[i])

        I = np.eye(dimP)
        Phi = I + A*dt 
        Adj = I
        Adj[0:dimP-dimTheta, 0:dimP-dimTheta] = Adjoint_SEK3(X)
        PhiAdj = Phi@Adj
        
        Qk_hat = PhiAdj@Qk@PhiAdj.transpose()*dt

        P_pred = Phi@P@Phi.transpose() + Qk_hat
        
        self.state_.setP(P_pred)
        


        
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
    inefk.propogate(np.random.randn(6,1), dt=0.1)

if __name__=="__main__":
    # testInEFKInit()
    testInEKFPropogate()