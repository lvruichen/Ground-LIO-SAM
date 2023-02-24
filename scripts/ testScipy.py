from scipy.spatial.transform import Rotation as R
import numpy as np

# r1 = R.from_quat([-0.00012385573858492355, 0.0001977789154841865, 0.7349529549630558, 0.6781180572247941])
# print(r1.as_matrix())
# r2 = R.from_matrix([[0, -1, 0], [1, 0, 0], [0, 0, 1]])

# r3 = R.from_matrix([[0, -1, 0], [1, 0, 0], [0, 0, 1]])

# r4 = r3 * r2

# r5 = R.from_matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

# # print(type(r5.inv().as_matrix()))

# a1 = np.ones((4,4))
# r7 = np.array([[0,1,0,0], [-1,0,0,0], [0,0,1,0], [0,0,0,1]])
# print(r7)
# r8 = np.array([[0,-1,0,5], [1,0,0,0], [0,0,1,0], [0,0,0,1]])

r1 = R.from_matrix([[0, -1, 0], [1, 0, 0], [0, 0, 1]])


r2 = R.from_matrix([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])










