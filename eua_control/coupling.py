from __future__ import division, print_function

import numpy as np


t1 = 1.5632
t2 = 1.0186
t3 = 1.2177
t4 = t3
w = 0.6089

K = np.array([
    [t1,  0,   0,   0   ],
    [0,   t2,  0,   0   ],
    [0,   -t2*w,   t3,  0   ],
    [0,   t2*w,   0,   -t4  ],
])
K2 = np.array([
    [t1,  0,   0,   0   ],
    [0,   t2,  0,   0   ],
    [0,   t2*w,   t3,  0   ],
    [0,   -t2*w,   0,   t4  ],
])
# K = np.array([
#     [t1,  0,   0,   0   ],
#     [0,   t2,  0,   0   ],
#     [0,   t2*w,   t3,  0   ],
#     [0,   -t2*w,   0,   t4  ],
# ])

print(K)
print(K2)


print('-'*78)

# d = np.array([0, 45, 45, 0])
# j = K.dot(d)
# print(j)

# print('-'*78)

# d = np.array([0, -80, 45, -135])
d = np.array([30, -75, 40, -130])
j = K.dot(d)
print(j)

d = np.array([30, -75, 40, 130])
print(K2.dot(d))

# print(np.linalg.solve(K, j))
# d = np.linalg.solve(K, j)

# j = np.array([0, 0, 115, 115])
# print(np.linalg.solve(K, j))


# j = np.array([0, 80, 115, 115])
# print(np.linalg.solve(K, j))


# j = np.array([0, -80, 115, 115])
# print(np.linalg.solve(K, j))

