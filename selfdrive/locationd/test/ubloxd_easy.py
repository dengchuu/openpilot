#!/usr/bin/env python
import os
import time
from selfdrive.locationd.test import ublox
from common import realtime
from selfdrive.locationd.test.ubloxd import gen_raw, gen_solution
import zmq
import selfdrive.messaging as messaging
from selfdrive.services import service_list
from selfdrive.car.tesla.readconfig import read_config_file,CarSettings


unlogger = os.getenv("UNLOGGER") is not None   # debug prints

def main(gctx=None):
  context = zmq.Context()
  poller = zmq.Poller()

  if not CarSettings().get_value("useTeslaGPS"):
    context = zmq.Context()
    gpsLocationExternal = messaging.pub_sock(context, service_list['gpsLocationExternal'].port)
    ubloxGnss = messaging.pub_sock(context, service_list['ubloxGnss'].port)

    # ubloxRaw = messaging.sub_sock(context, service_list['ubloxRaw'].port, poller)

    # buffer with all the messages that still need to be input into the kalman
    while 1:
      polld = poller.poll(timeout=1000)
      for sock, mode in polld:
        if mode != zmq.POLLIN:
          continue
        logs = messaging.drain_sock(sock)
        for log in logs:
          buff = log.ubloxRaw
          ttime = log.logMonoTime
          msg = ublox.UBloxMessage()
          msg.add(buff)
          if msg.valid():
            if msg.name() == 'NAV_PVT':
              sol = gen_solution(msg)
              if unlogger:
                sol.logMonoTime = ttime
              else:
                sol.logMonoTime = int(realtime.sec_since_boot() * 1e9)
              gpsLocationExternal.send(sol.to_bytes())
            elif msg.name() == 'RXM_RAW':
              raw = gen_raw(msg)
              if unlogger:
                raw.logMonoTime = ttime
              else:
                raw.logMonoTime = int(realtime.sec_since_boot() * 1e9)
              ubloxGnss.send(raw.to_bytes())
          else:
            print "INVALID MESSAGE"
  else:
    while True:
      time.sleep(1.1)


if __name__ == "__main__":
  main()
