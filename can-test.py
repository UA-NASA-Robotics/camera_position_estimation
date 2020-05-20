import serial
import can

bus = can.interface.Bus(bustype='seeedstudio', channel='/dev/ttyUSB13', bitrate=500000)
     

msg = can.Message(arbitration_id=0x00000001,
                  data=[0, 0, 0, 0, 0, 0, 0, 0],
                  is_extended_id=True)
while(1):

    try:
        bus.send(msg)
        print("Message sent on {}".format(bus.channel_info))
    except can.CanError:
        print("Message NOT sent")
