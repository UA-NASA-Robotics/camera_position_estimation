import can
import time 

# Candlelight firmware on Linux
bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=500000)

# Stock slcan firmware on Linux
#bus = can.interface.Bus(bustype='slcan', channel='/dev/ttyACM5', bitrate=500000)

# Stock slcan firmware on Windows
#bus = can.interface.Bus(bustype='slcan', channel='COM0', bitrate=500000)

msg = can.Message(arbitration_id=0xc0ffee,
                  data=[0, 25, 0, 1, 3, 1, 4, 1],
                  is_extended_id=True)

while(1):
  time.sleep(500)
  try:
      bus.send(msg)
      print("Message sent on {}".format(bus.channel_info))
  except can.CanError:
      print("Message NOT sent")