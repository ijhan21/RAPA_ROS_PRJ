import serial
ser = serial.Serial("/dev/ttyACM0", 115200)
while True:
    msg = ser.read().decode()
    print(msg, end="")
    if msg =='*':print()

