from scipy.spatial.transform import Rotation as R
import numpy as np

r1 = R.from_quat([-0.00012385573858492355, 0.0001977789154841865, 0.7349529549630558, 0.6781180572247941])
# print(r.as_matrix())
r2 = R.from_matrix([[0, -1, 0], [1, 0, 0], [0, 0, 1]])

r3 = R.from_matrix([[0, -1, 0], [1, 0, 0], [0, 0, 1]])

r4 = r3 * r2

r5 = R.from_matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

print(type(r5.inv().as_matrix()))
print(r5.as_matrix())



