import numpy as np
import copy
'''

'''
class RobotState:
    def __init__(self):
        # X_t 
        # SE_N+2 [R, v, p]
        self.X_ = np.eye(5)
        # Theta_t
        # [bg, ba]
        self.Theta_ = np.zeros((6, 1))
        self.P_ = np.eye(15)
    
    def getX(self):
        return copy.deepcopy(self.X_)
    
    def getTheta(self):
        return  copy.deepcopy(self.Theta_)

    def getP_(self):
        return copy.deepcopy(self.P_)

    def getRotation(self):
        R = self.X_[:3, :3]
        return copy.deepcopy(R)
    
    def getPosition(self):
        p = self.X_[:3, 4] #（3，）
        return copy.deepcopy(p)
    
    def getVelocity(self):
        v = self.X_[:3, 3] #（3，）
        return copy.deepcopy(v)

#----------------------------------------------for testing
def testRobotStateInit():
    state = RobotState()
    Xt = state.getX()
    Theta = state.getTheta()
    P = state.getP_()
    print(Xt)
    print(Theta)
    print(P)
    print(state.getRotation())
    position = state.getPosition()
    print(position.shape)
    print(state.getVelocity())


if __name__=="__main__":
    testRobotStateInit()
    
