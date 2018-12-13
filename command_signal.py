import zmq
import time

fs = 5
T = 1/fs
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://127.0.0.1:5557")
counter = 1

while True:
    value = 1
    socket.send_pyobj([value, counter])
    counter = counter + 1
    print(value)
    time.sleep(T)
