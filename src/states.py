import numpy as np

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
        return self.X_
    
    def getTheta(self):
        return self.Theta_

    def getP_(self):
        return self.P_

#----------------------------------------------for testing
def testRobotStateInit():
    state = RobotState()
    Xt = state.getX()
    Theta = state.getTheta()
    P = state.getP_()
    print(Xt)
    print(Theta)
    print(P)


if __name__=="__main__":
    testRobotStateInit()
    
