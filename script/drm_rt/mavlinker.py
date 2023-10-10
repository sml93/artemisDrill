from pymavlink import mavutil

master = mavutil.mavlink_connection('udp:localhost:14445')

print("waiting for heartbeat...")
master.wait_heartbeat()

msg_names = master.messages.keys()


print("Available MAVLink messages:")
for name in msg_names:
  print("- {}".format(name))
print(master.messages['ATTITUDE'].roll)
