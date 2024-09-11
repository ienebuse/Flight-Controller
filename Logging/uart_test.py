import serial
import signal


ser = serial.Serial('COM16', 256000)
while True:
    resp = ser.read_until()
    val = resp.decode().strip().split(',')
    print(f'                                                                       ', end='\r')
    try:
        # print(f'{val[0]}\t{val[1]}\t{val[2]}\t{val[3]}\t{val[5]}\t{val[4]}\t{val[6]}    ', end='\r')
        print(f'{val[0]}\t\t{val[1]}\t\t{val[2]}    ', end='\r')    
    except:
        continue