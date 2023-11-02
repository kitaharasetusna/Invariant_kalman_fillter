import numpy as np

# resize the shape of a matrix by returning a new matrix
# all the padding will be 0
def resizeNdarray(ori_matrix, desired_shape):
    pad_rows = desired_shape[0] - ori_matrix.shape[0]
    pad_cols = desired_shape[1] - ori_matrix.shape[1]
    # return a new matrix (padded)
    pad_matrix = np.pad(ori_matrix, ((0, pad_rows), (0, pad_cols)), mode='constant')
    return pad_matrix




#----------------------------------------------for testing
def test_resizeNdarray():
    randn_matrix = np.random.randn(6,7)
    print(randn_matrix)
    resized_matrix = resizeNdarray(ori_matrix=randn_matrix, desired_shape=(8, 9))
    print(resized_matrix)
    print(randn_matrix.shape)
    
if __name__=="__main__":
    test_resizeNdarray()
