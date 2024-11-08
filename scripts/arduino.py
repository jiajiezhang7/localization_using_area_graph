import serial 
port = "/dev/ttyUSB0"
ser = serial.Serial(port,9600,timeout=1) 
ser.flushInput()

try:
	while True:
		ser.write(b'1') #
		response = ser.read()
		print(var(response))
except:
	print("connect fail")
	ser.close()	#
