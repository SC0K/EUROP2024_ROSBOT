
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import numpy as np
import pyomo.environ as pyo
from pyomo.opt import SolverFactory

X = np.zeros((4, 2))
state = np.ones(2)
targets = np.array([[5, 0, 0, 0],
                    [-5, 0, 0, 0],
                    ]).T
print(targets[:,0])
state1 = np.array([1, 2, 3, 4])
state2 = np.array([5, 6, 7, 8])
states = np.array([state1,state2])
print(f"states: {states}, 0,0: {states[0,0]}")

for d, state in enumerate([state1, state2]):
    print(f"Index: {d}, State: {state}")

combinations = np.array([np.delete(np.arange(2), i) for i in range(2)])
print(combinations)
print(range(combinations.shape[1]))

model = pyo.ConcreteModel()
model.X = pyo.Var(range(5+1), range(4))


Bd = {j: 5/1000*(-1)**(j+1)*np.vstack([np.eye(2), np.zeros((2, 2))]) for j in range(1, 2+1)}
print(Bd)
print(Bd[1][1,1])

disturbance = np.zeros((2, 5, 2))
for d in range(2):
    for n in range(5):
        dis = 0 + 1 * np.random.randn(2, 119)
        disturbance[:, n, d] = np.max(dis, axis=1)
print(disturbance)