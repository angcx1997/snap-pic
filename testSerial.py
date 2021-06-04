import serial, time


arduino = serial.Serial('COM5', 115200, timeout=.1)
time.sleep(1) #give the connection a second to settle
# arduino.write("Hello from Python!")
while True:
	data = arduino.readline()[0:-2]
	print(data)