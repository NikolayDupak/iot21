#!/usr/bin/python3

import time
from Mine import Mine

# ---------------------------
print("Start")


mine = Mine()

try:
    mine.client.loop_start()
    t = time.time()
    while True:
        mine.update()
        if time.time() - t > 30:
            if mine.connect is False:
                mine.try_connect()
            else:
                mine.send_temp_humi()
                t = time.time()


except KeyboardInterrupt:
    print("Stop.")
finally:
    mine.delete()
    mine.client.loop_stop()
    mine.client.disconnect()
