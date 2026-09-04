import sys
import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

sys.path.append("..")
from scservo_sdk import *

# Default setting
SCS_ID = 1            # SCServo ID : 1
NEW_ID = 2             #Change the Servo ID
BAUDRATE = 1000000     #SCServo default baudrate:1000000
DEVICENAME = 'COM53'   # Check which port is being used on your controller
                       # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Get methods and members of Protocol
packetHandler = sms_sts(portHandler)
# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to set the baudrate")
else:
    print("Failed to set the baudrate")
    portHandler.closePort()
    quit()


scs_comm_result, scs_error = packetHandler.unLockEprom(SCS_ID)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))
    getch()
    quit()

scs_comm_result, scs_error = packetHandler.write1ByteTxRx(SCS_ID, scs_id, NEW_ID)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
else:
    packetHandler.LockEprom(SCS_ID)
    print("Succeeded to change the Servo ID")
if scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))
    getch()
    quit()