import matplotlib.pyplot as plt
import numpy
import zmq

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://127.0.0.1:5558")
socket.setsockopt_string(zmq.SUBSCRIBE, '')

hl, = plt.plot([], [])

def update_line(hl, new_data):
    hl.set_xdata(numpy.append(hl.get_xdata(), new_data))
    hl.set_ydata(numpy.append(hl.get_ydata(), new_data))
    plt.draw()

while True:
    received_msg = socket.recv_pyobj()
    update_line(received_msg[0], received_msg[1])
