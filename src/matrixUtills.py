import numpy as np

TOL_EXP = 1e-10

# resize the shape of a matrix by returning a new matrix
# all the padding will be 0
def resizeNdarray(ori_matrix, desired_shape):
    pad_rows = desired_shape[0] - ori_matrix.shape[0]
    pad_cols = desired_shape[1] - ori_matrix.shape[1]
    # return a new matrix (padded)
    pad_matrix = np.pad(ori_matrix, ((0, pad_rows), (0, pad_cols)), mode='constant')
    return pad_matrix

def skew(v):
    M = np.zeros((3,3))
    M[0, 1] = -v[2,0]; M[0, 2] = v[1,0]
    M[1, 0] = v[2,0]; M[1, 2] = -v[0,0]
    M[2, 0] = -v[1,0]; M[2, 1] = v[0,0]
    return M
    

def EXPSO3(w):
    A = skew(w)
    theta = np.linalg.norm(A)
    global TOL_EXP
    if theta<TOL_EXP:
       return np.eye(3) 
    # Eular angle to rotation matrix
    R = np.eye(3)+(np.sin(theta)/theta)*A + ((1-np.cos(theta))/(theta*theta))*A@A
    return R 

# v:(n,1)
def EXPSE3(v):
    K = (v.shape[0]-3)/3
    X = np.eye(K+3)
    w = v[:3]
    theta = np.linalg.norm(w)
    I = np.eye(3)
    if theta<TOL_EXP:
        R = I
        Jl = I
    else:
        A = skew(w)
        theta2 = theta*theta
        stheta = np.sin(theta)
        ctheta = np.cos(theta)
        oneMinusCosTheta2 = (1-ctheta)/(theta2)
        A2 = A@A
        R =  I + (stheta/theta)*A + oneMinusCosTheta2*A2
        Jl = I + oneMinusCosTheta2*A + ((theta-stheta)/(theta2*theta))*A2
    X[:3, :3] = R
    for i in range(K):
        X[:3, 3+i] = Jl@v[3+3*i:6+3*i]
    return X


def Adjoint_SEK3(X):
    K = X.shape[1]-3 # num([v, p, d])
    Adj = np.zeros((3+3*K, 3+3*K))
    R = X[:3, :3]
    Adj[:3,:3] = R
    for i in range(K):
        Adj[3+3*i:6+3*i,3+3*i:6+3*i] = R
        Adj[3+3*i:6+3*i,0:3]= skew(X[:3, 3+i].reshape(-1, 1))@R
    return Adj
    

#----------------------------------------------for testing
def test_resizeNdarray():
    randn_matrix = np.random.randn(6,7)
    print(randn_matrix)
    resized_matrix = resizeNdarray(ori_matrix=randn_matrix, desired_shape=(8, 9))
    print(resized_matrix)
    print(randn_matrix.shape)

def test_skew():    
    v_test = np.array([0.1,1,2]).reshape(-1,1)
    v_x = skew(v_test)
    print(v_x)

def test_EXPSO3():
    R = EXPSO3(np.random.randn(4,1))
    print(R)

if __name__=="__main__":
    # test_resizeNdarray()
    # test_skew()
    test_EXPSO3()
