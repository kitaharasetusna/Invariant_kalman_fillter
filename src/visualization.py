import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

def VisualizeSE3(se3_matrix):
    position = se3_matrix[:3, 3]
    rotation_matrix = se3_matrix[:3, :3]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the coordinate axes representing orientation
    axis_length = 1.0  # Length of the coordinate axes
    axes_colors = ['r', 'g', 'b']  # X, Y, and Z axes colors

    for i in range(3):
        # Direction vectors for the axes
        direction = rotation_matrix[:, i]
        end_point = position + axis_length * direction
        ax.quiver(position[0], position[1], position[2], direction[0], direction[1], direction[2], color=axes_colors[i])

    # Set axis labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Set plot limits (adjust as needed)
    ax.set_xlim([-10, 10])
    ax.set_ylim([-10, 10])
    ax.set_zlim([-10, 10])

    # Show the plot
    plt.show()



def VisulizeSE3Anim(se3_matrices):
    def update(frame):
        ax.cla()

        # Extract the current SE(3) matrix
        se3_matrix = se3_matrices[frame]

        # Extract the translation vector (position) from the SE(3) matrix
        position = se3_matrix[:3, 3]

        # Extract the rotation matrix (orientation) from the SE(3) matrix
        rotation_matrix = se3_matrix[:3, :3]

        # Plot the path of the object
        path = np.array([m[:3, 3] for m in se3_matrices[:frame+1]])
        ax.plot(path[:, 0], path[:, 1], path[:, 2], color='b', linestyle='-')

        # Plot the coordinate axes representing orientation
        axis_length = 1.0  # Length of the coordinate axes
        axes_colors = ['r', 'g', 'b']  # X, Y, and Z axes colors

        for i in range(3):
            direction = rotation_matrix[:, i]
            end_point = position + axis_length * direction
            ax.quiver(position[0], position[1], position[2], direction[0], direction[1], direction[2], color=axes_colors[i])

        # Set axis labels
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        ax.set_xlim([-100, 100])
        ax.set_ylim([-100, 100])
        ax.set_zlim([-100, 100])

    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
   
    # Create the animation
    animation = FuncAnimation(fig, update, frames=len(se3_matrices), repeat=False)
    plt.show() 

#----------------------------------------------for testing
def testVisualizeSingleFrame():
    # Example SE(3) matrix (4x4)
    se3_matrix = np.array([
        [0.866, -0.5, 0.0, 1.0],
        [0.5, 0.866, 0.0, 2.0],
        [0.0, 0.0, 1.0, 3.0],
        [0.0, 0.0, 0.0, 1.0]
    ])
    VisualizeSE3(se3_matrix=se3_matrix)


def gen_se3_matrics():
    se3_matrix = np.array([
        [0.866, -0.5, 0.0, 1.0],
        [0.5, 0.866, 0.0, 2.0],
        [0.0, 0.0, 1.0, 3.0],
        [0.0, 0.0, 0.0, 1.0]
    ])
    se3_matrix*=10
    matrices = []
    matrices.append(se3_matrix)
    last_mat = se3_matrix
    for _ in range(100):
       next_mat = last_mat.copy() 
       next_mat[:3, -1] = next_mat[:3, -1]+1
       matrices.append(next_mat)
       last_mat = next_mat
    return matrices

def testViusualizeSE3Anim():
    se3_matrices = gen_se3_matrics() 
    VisulizeSE3Anim(se3_matrices)


if __name__=="__main__":
    #testVisualizeSingleFrame()
    testViusualizeSE3Anim()