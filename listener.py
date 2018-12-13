import time
import zmq

context = zmq.Context()
filter = "coords"
socket = context.socket(zmq.SUB)
socket.connect("tcp://127.0.0.1:5556")
socket.setsockopt_string(zmq.SUBSCRIBE, '')


print("Subscribing to topic: \"coords\"")
while True:
    message = socket.recv_pyobj()
    print("x: %f, y: %f, z: %f" % (message[1], message[2], message[3]))
    # time.sleep(0.5)
