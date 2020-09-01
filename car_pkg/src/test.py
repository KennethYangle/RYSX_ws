from pymavlink import mavutil
import time

master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)# port 是端口号
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_system))

cnt = 0
start_time = time.time()
while True:
    try:
        msg = master.recv_match(type=['GPS_RAW_INT'], blocking=True)
        if not msg:
            raise ValueError()
        print(msg.to_dict())
        cnt += 1
        if cnt % 10 == 0:
            print("rate: {}".format(cnt/(time.time()-start_time)))
    except KeyboardInterrupt:
        print('Key bordInterrupt! exit')
        break