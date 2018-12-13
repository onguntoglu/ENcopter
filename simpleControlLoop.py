import control
import zmq
import numpy as np
import scipy
import matplotlib.pyplot as plt
import PID

# Create networking for publisher
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://127.0.0.1:5557")
socket.setsockopt_string(zmq.SUBSCRIBE, '')

# Create networking for PUSH to plotting script
socket_push = context.socket(zmq.PUB)
socket_push.bind("tcp://127.0.0.1:5558")

# Create PID controller
Kp = 1
Kd = 1
Ki = 1
pid = PID.PID(Kp, Ki, Kd, 0)
pid.setKp(6)

A = np.matrix('[2 -3; 1 3]')
B = np.matrix('[1;-1]')
C = np.matrix('[1 0]')
D = 0

A2 = np.matrix('[0 1; -2 -3]')
B2 = np.matrix('[0;-1]')
C2 = np.matrix('[1 0]')
D2 = 0

sys = control.ss(A, B, C, D)
tf = control.ss2tf(sys)
poles = control.pole(sys)
acker_k = control.acker(A, B, ["-1", "-1"])

sys_stabil = control.ss(A2, B2, C2, D2)

Acl = A-B*acker_k

sys2 = control.ss(Acl, B, C, D)
pid_ss = pid.create_tf()
sys3 = control.series(sys2, pid_ss)
sys4 = control.feedback(sys3, 1, -1)

while True:
    value = socket.recv_pyobj()
    sys_step = control.tf(value[0], [0, 1])
    sys_command = control.series(sys_step, sys4)
    T, yout = control.step_response(sys_command)
    socket_push.send_pyobj([T, yout])
