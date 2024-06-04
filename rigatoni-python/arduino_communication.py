import serial.tools.list_ports

ardSerialInst = serial.Serial()

### Manual port finding at time of run
ports = serial.tools.list_ports.comports()

portsList = []

for onePort in ports:
    portsList.append(str(onePort))
    print(str(onePort))

val = input("Select Port: COM")

for x in range(0, len(portsList)):
    if portsList[x].startswith("COM" + str(val)):
        portVar = "COM" + str(val)
        print(portVar)

## end of manual port findng at time of run

#portVar = "COM7"
ardSerialInst.baudrate = 9600
ardSerialInst.port = portVar
ardSerialInst.open()

while True:
    command = input("Arduino Command (OPEN/CLOSE):")
    ardSerialInst.write(command.encode('utf-8'))
