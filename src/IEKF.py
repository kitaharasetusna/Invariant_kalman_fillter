import numpy as np
from states import RobotState
from NoiseParam import NoiseParam
from matrixUtills import EXPSO3, skew, Adjoint_SEK3, resizeNdarray, EXPSE3

class Kinematics:
    def __init__(self, id, pose, covariance):
        self.id_ = id
        self.pose_ = pose
        self.covariance_ = covariance

class Observation:
    def __init__(self, Y, b, H, N, PI):
        self.Y_ = Y
        self.b_ = b
        self.H_ = H
        self.N_ = N
        self.PI_  = PI

def removeRowAndColumn(M, index):
    dimX = M.shape[1] 
    M[index:dimX-1 , :] = M[index+1:dimX, :]
    M[:, index:dimX-1] = M[:, index+1, dimX]
    new_M = np.zeros((dimX-1, dimX-1)) 
    new_M = M[:dimx-1, :dimX-1]
    return new_M

class InEKF:
    def __init__(self, state, params):
        self.g_ = np.array([0, 0, -9.81]).reshape(-1,1)
        self.state_ = state
        self.noise_params_ = params
        self.estimated_contact_positions_ = {}
        self.contacts_ = {}
    
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
    
    # contacts [[id, {0/1}], ...]
    def setContacts(self, contacts):
        for it in contacts:
            self.contacts_[it[0]] = it[1]
   
   # measured_kinematics: a list to Kinematics 
    def CorrectKinematics(self, measured_kinematics): 
        R = self.state_.getRotation()
        remove_contacts = []
        new_contacts = []
        Y = np.empty((0, 0))
        b = np.empty((0, 0))
        H = np.empty((0, 0))
        N = np.empty((0, 0))
        PI = np.empty((0, 0))
        
        for it in measured_kinematics:
            if it.id_ in self.contacts_.keys():
                contact_indicated = self.contacts_[it.id_]
            else:
                continue

            if it.id_ in self.estimated_contact_positions_.keys():
                found = True
            else:
                found = False
            
            # if current censor have no contact events 
            # and we have update the contact in X_t
            # remove it
            if contact_indicated==False and found==True:
                # (id, start_index)
                # TODO: check this
                remove_contacts.append([it.id_, self.estimated_contact_positions_[it.id_]])
            elif contact_indicated==True and found==False:
                # if we haven't seen this before and it have contact event(1)
                # put it into the new contacts
                new_contacts.append(it) 
            elif contact_indicated==True and found==True:
                # TODO: check this
                # after updating P and X, use the updated Y, b, H, PI
                # in this section, for convenience, we used index instead
                dimX = self.state_.dimX()
                dimP = self.state_.dimP()
                
                startIndex = Y.shape[0]
                Y = resizeNdarray(Y, (startIndex+dimX, 1))
                # [h_p(alpha_t), 0, 1, -1]
                Y[startIndex:startIndex+3] = it.pose_[0:3, 3]
                Y[startIndex+4]  = 1
                # TODO: check wether the index it's 5
                T[startIndex+self.estimated_contact_positions_[it.id_]] = -1

                # [0_{1,3}, 0, -1, 1]
                startIndex = b.shape[0]
                b = resizeNdarray(b, (startIndex+dimX, 1)) 
                b[startIndex+4] = 1
                b[startIndex+self.estimated_contact_positions_[it.id_]] = -1

                # [0_{3,3}, 0_{3,3}, -I, I, 0_{3,3} 0_{3,3}]
                startIndex = H.shape[0]
                H = resizeNdarray(H, (startIndex+3, dimP))
                H[startIndex:startIndex+3, 6:9] = -np.eye(3)
                H[startIndex:startIndex+3,  \
                    3*self.estimated_contact_positions_[it.id_]-6:3*self.estimated_contact_positions_[it.id_]-3] \
                        = np.eye(3)
                
                startIndex = N.shape[0]
                N = resizeNdarray(N, (startIndex+3, startIndex+3))
                N[startIndex:startIndex+3, startIndex:startIndex+3] =  R@it.covariance_[3:6, 3:6]@R.T

                startIndex = PI.shape[0]
                startIndex_col = PI.shape[1]
                PI = resizeNdarray(PI, (startIndex+3, startIndex_col+dimX))
                PI[startIndex:startIndex+3, startIndex_col:startIndex_col+3] = np.eye(3)
            else:
                continue

            obs = Observation(Y=Y, b=b, H=H, N=N, PI=PI) 
            if Y.shape[0]>0:
                # when you have contact information and
                # we have already update P and Q, then do correction
                self.Correct(obs)
            
            # removing unnecessary contacts from P and X
            if len(remove_contacts)>0:
                X_rem = self.state_.getX() 
                P_rem = self.state_.getP_()
                for it in remove_contacts:
                    # Remove from list of estimated contact positions
                    del self.estimated_contact_positions_[it[0]]
                    # TODO: change this into matrix multiplications of F (after we test this part on the txt file)
                    X_rem = removeRowAndColumn(M=X_rem, index=it[1])

                    startIndex = 3+3*(it[1]-3)
                    P_rem = removeRowAndColumn(M=P_rem, index=startIndex)
                    P_rem = removeRowAndColumn(M=P_rem, index=startIndex)
                    P_rem = removeRowAndColumn(M=P_rem, index=startIndex)

                    # TODO: update landmarks

                    # update estimation
                    for it2 in self.estimated_contact_positions_.keys():
                        if self.estimated_contact_positions_[it2] > it[1]:
                            self.estimated_contact_positions_[it2] -=1
                    
                    # in case we have to remove multiple contacts
                    for it2 in remove_contacts:
                        if it2[1]>it[1]:
                            it2[1] -= 1
                    
                    self.state_.setX(X_rem)
                    self.state_.setP(P_rem)
            
            
             
    def Correct(self, obs):
        # St = Ht @ Pt @Ht.T() + Nt
        # Kt = Pt @ Ht.T @St^{-1}
        P = self.state_.getP_()

        PHT = P@obs.H_.T
        S = obs.H_ @PHT+obs.N_

        K = PHT@np.linalg.inv(S)

        BigX_ori = np.empty((0, 0))
        # TODO: check this division
        # repeat X for number of observations(contact events)
        BigX = self.state_.copyDiag(n=obs.Y_.shape[0]/self.state_.dimX(), BigX=BigX_ori)
        
        # XY-b
        Z = BigX@obs.Y_ - obs.b_
        delta = K@obs.PI_@Z
        # got the lie algebra for the update
        dX = Exp_SEK3(delta[0:delta.shape[0]-state_.dimTheta()])
        # dTheta = delta.segment(delta.rows()-state_.dimTheta(), state_.dimTheta());
        dtheta = delta[delta.shape[0]-self.state_.dimTheta():delta.shape[0]]

        # Xt^{+} = exp ( Lt (hat(Xt)Yt − b )) @Xt
        X_new = dX@slef.state_.getX()
        # Theta_new = state_.getTheta() + dTheta;
        Theta_new = self.state_.getTheta()+dtheta

        self.state_.setX(X=X_new)
        self.state_.setTheta(theta=Theta_new)

        # Update Covariance
        # Pt_new = (I − Kt@Ht)@Pt
        IKH = np.eye(self.state_.dimP()) - K@obs.H_
        # however, we used Joseph update form: more stable
        P_new = IKH@P@IKH.T+K@obs.N_@K.T 
        #Eigen::MatrixXd P_new = IKH * P * IKH.transpose() + K*obs.N*K.transpose(); // Joseph update form

        self.state_.setP(P_new)

        



            
                    


        
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