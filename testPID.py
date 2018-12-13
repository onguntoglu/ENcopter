import PID

pid = PID.PID(2, 1, 1)
sys = pid.tf()
print(sys)