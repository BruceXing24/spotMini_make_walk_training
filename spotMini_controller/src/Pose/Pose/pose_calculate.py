# initial angle is
# initail position
# [[ 0.          0.          0.          0.        ]
#  [ 0.01248707 -0.01248707  0.01248707 -0.01248707]
#  [-0.28935    -0.28935    -0.28935    -0.28935   ]]
import numpy as np

h = 0.16332    # 0.2835
w = 0.169545   # 0.222
l = 0.187687   # 0.38
b = w-0.09     # 0.093
# these data from measurement in pubullet, check contact point

def get_AB(r,  p , y ):
    posi = np.mat([0.0, 0.0,  h]).T
    rpy = np.array([r, p, y]) * np.pi/180

    R, P, Y = rpy[0], rpy[1], rpy[2]
    rotx = np.mat([[ 1,       0,            0     ],
                  [ 0,       np.cos(R), -np.sin(R)],
                  [ 0,       np.sin(R),  np.cos(R)]])

    roty = np.mat([[ np.cos(P),  0,      -np.sin(P)],
                   [ 0,            1,       0      ],
                   [ np.sin(P),  0,       np.cos(P)]])

    rotz = np.mat([[ np.cos(Y), -np.sin(Y),  0     ],
                   [ np.sin(Y),  np.cos(Y),  0     ],
                   [ 0,            0,        1     ]])
    rot_mat = rotx * roty * rotz
    body_stru = np.mat([
        [l/2,    b/2,   0],
        [l/2,   -b/2,   0],
        [-l/2,   b/2,   0],
        [-l/2,  -b/2,   0] ]).T

    footint_struc = np.mat([[   l / 2,  w / 2,  0],
                              [ l / 2, -w/ 2,  0],
                              [-l / 2,  w / 2,  0],
                              [-l / 2, -w / 2,  0]]).T

    AB = np.mat( np.zeros((3,4)) )
    for i in range(4):
        AB[:,i] = -posi - rot_mat * body_stru[:,i] + footint_struc[:,i]
    return AB





if __name__ == '__main__':
    pass
    ## simple test
    # theta1, theta2 ,theta3= inverkinematic(0, 0.012 ,0.2835 )
    # print("theta1==={}, theta2==={},theta3=={}".format(theta1*180/np.pi, theta2*180/np.pi ,theta3*180/np.pi))