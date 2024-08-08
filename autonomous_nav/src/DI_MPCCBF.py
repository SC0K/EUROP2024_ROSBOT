#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import numpy as np
import pyomo.environ as pyo
from pyomo.opt import SolverFactory

class AccelToCmdVel:
    def __init__(self):
        rospy.init_node('accel_to_cmd_vel')

        self.h = 0.1
        self.A0 = np.array([[1, 0, self.h, 0],
                            [0, 1, 0, self.h],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
        self.B0 = np.array([[self.h**2/2, 0],
                            [0, self.h**2/2],
                            [self.h, 0],
                            [0, self.h]])
        self.nx = 4  # Number of states
        self.nu = 2  # Number of inputs
        self.nd = 2  # Number of drones

        self.m = 119  # Number of scenarios
        self.r1 = 0.5  # Drone proximity limits
        self.r2 = 0.5
        self.gamma = 0.2
        self.a_lim = 4  # Acceleration limit (m/s^2)

        self.X = np.zeros((self.nx, self.nd))
        self.targets = np.array([[5, 0, 0, 0],
                                 [-5, 0, 0, 0],
                                 ]).T

        self.Q = 5 * np.eye(self.nx)
        self.R = np.eye(self.nu)
        self.eta = 0.1
        self.N = 5  # MPC Horizon

        self.mu = 0
        self.sigma = 1
        np.random.seed(1234)
        self.Bd = {j: 5/1000*(-1)**(j+1)*np.vstack([np.eye(2), np.zeros((2, 2))]) for j in range(self.nd)}

        self.disturbance = np.zeros((2, self.N, self.nd))
        for d in range(self.nd):
            for n in range(self.N):
                dis = self.mu + self.sigma * np.random.randn(2, self.m)
                self.disturbance[:, n, d] = np.max(dis, axis=1)
        
        self.combinations = np.array([np.delete(np.arange(self.nd), i) for i in range(self.nd)])

        # ROS Subscribers
        self.odom_sub1 = rospy.Subscriber('/robot1/odom', Odometry, self.odom_callback1)
        self.odom_sub2 = rospy.Subscriber('/robot2/odom', Odometry, self.odom_callback2)

        # ROS Publishers for cmd_vel
        self.cmd_accel_x_pub1 = rospy.Publisher('/robot1/accel_x', Float32, queue_size=10)
        self.cmd_accel_y_pub1 = rospy.Publisher('/robot1/accel_y', Float32, queue_size=10)
        self.cmd_accel_x_pub2 = rospy.Publisher('/robot2/accel_x', Float32, queue_size=10)
        self.cmd_accel_y_pub2 = rospy.Publisher('/robot2/accel_y', Float32, queue_size=10)

        self.rate = rospy.Rate(10)

        # Initializing states for the two robots
        self.state1 = np.zeros(self.nx)
        self.state2 = np.zeros(self.nx)

        self.run()

    def odom_callback1(self, msg):
        self.state1[0] = msg.pose.pose.position.x
        self.state1[1] = msg.pose.pose.position.y
        self.state1[2] = msg.twist.twist.linear.x
        self.state1[3] = msg.twist.twist.linear.y

    def odom_callback2(self, msg):
        self.state2[0] = msg.pose.pose.position.x
        self.state2[1] = msg.pose.pose.position.y
        self.state2[2] = msg.twist.twist.linear.x
        self.state2[3] = msg.twist.twist.linear.y

    def mpc_optimization(self, X, target, A0, B0, Bd, disturbance, Q, R, eta, N, r1, r2, gamma, a_lim, d):
        model = pyo.ConcreteModel()

        # Variables
        model.u = pyo.Var(range(N), range(self.nu), bounds=(-a_lim, a_lim))
        model.X = pyo.Var(range(N+1), range(self.nx))

        # Constraints
        def state_update(model, k):
            if k == 0:
                return [model.X[0, i] == X[d,i] for i in range(self.nx)]
            else:
                return [model.X[k, i] == sum(A0[i, j] * model.X[k-1, j] for j in range(self.nx)) +
                        sum(B0[i, j] * model.u[k-1, j] for j in range(self.nu)) +
                        sum(Bd[d][i, j] * disturbance[j, k-1, d] for j in range(2))
                        for i in range(self.nx)]

        model.state_constraints = pyo.ConstraintList()
        for k in range(N+1):
            for con in state_update(model, k):
                model.state_constraints.add(con)

        model.input_constraints = pyo.ConstraintList()
        for k in range(N):
            model.input_constraints.add(-a_lim <= model.u[k, 0])
            model.input_constraints.add(model.u[k, 0] <= a_lim)
            model.input_constraints.add(-a_lim <= model.u[k, 1])
            model.input_constraints.add(model.u[k, 1] <= a_lim)

        # Barrier function constraints
        for c in range(self.combinations.shape[1]-1):
            for k in range(N):
                hk = (model.X[k, 0] - X[self.combinations[d, c],0])**2/r1**2 + (model.X[k, 1] - X[self.combinations[d, c],1])**2/r2**2 - 1
                hk1 = (model.X[k+1, 0] - X[self.combinations[d, c],0])**2/r1**2 + (model.X[k+1, 1] - X[self.combinations[d, c],1])**2/r2**2 - 1
                model.state_constraints.add(hk1 - hk + gamma * hk >= 0)

        # Objective function
        def objective(model):
            return sum((model.X[k, i] - target[i])**2 * Q[i, i] for k in range(N+1) for i in range(self.nx)) + \
                   sum(model.u[k, i]**2 * R[i, i] for k in range(N) for i in range(self.nu))

        model.objective = pyo.Objective(rule=objective, sense=pyo.minimize)

        return model

    def run(self):
        while not rospy.is_shutdown():
            for d, state in enumerate([self.state1, self.state2]):
                states = np.array([self.state1, self.state2])
                target = self.targets[:, d]
                model = self.mpc_optimization(states, target, self.A0, self.B0, self.Bd, self.disturbance, self.Q, self.R, self.eta, self.N, self.r1, self.r2, self.gamma, self.a_lim, d)

                # Solve the optimization problem
                solver = SolverFactory('ipopt')
                result = solver.solve(model)

                if result.solver.status == pyo.SolverStatus.ok and result.solver.termination_condition == pyo.TerminationCondition.optimal:
                    rospy.loginfo(f'Drone {d+1}: Optimization successful.')
                else:
                    rospy.logwarn(f'Drone {d+1}: Optimization failed.')

                # Extract the control inputs
                u_opt = np.array([pyo.value(model.u[k, i]) for k in range(self.N) for i in range(self.nu)]).reshape(self.N, self.nu)
                self.X[:, d] = self.A0 @ self.X[:, d] + self.B0 @ u_opt[0] + self.Bd[d+1] @ (self.mu + self.sigma * np.random.randn(2))

                rospy.loginfo(f'Drone {d+1}: Control input: {u_opt[0]}')

                # Publish the control inputs
                if d == 0:
                    self.cmd_accel_x_pub1.publish(Float32(u_opt[0][0]))
                    self.cmd_accel_y_pub1.publish(Float32(u_opt[0][1]))
                else:
                    self.cmd_accel_x_pub2.publish(Float32(u_opt[0][0]))
                    self.cmd_accel_y_pub2.publish(Float32(u_opt[0][1]))

                # Check if the robot is close to its target
                distance_to_target = np.linalg.norm(self.X[:2, d] - target[:2])
                if distance_to_target < 0.1:
                    rospy.loginfo(f'Drone {d+1} reached the target.')
                    continue

            self.rate.sleep()

if __name__ == '__main__':
    try:
        AccelToCmdVel()
    except rospy.ROSInterruptException:
        pass
