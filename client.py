import zmq
import sys
import json
from cereal import log

port = 8018
#ipaddress = "192.168.1.33"
ipaddress = "192.168.1.33"
#ipaddress = "192.168.43.1"
src = ""

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect ("tcp://%s:%d" % (ipaddress, port))
socket.setsockopt(zmq.SUBSCRIBE, b"")
# myCan = None
poller = zmq.Poller()
poller.register(socket, zmq.POLLIN)

while 1:
  msg = []
  poller.poll()
  thisData = None
  while 1:
    try:
      thisData = socket.recv(zmq.NOBLOCK)
    except zmq.error.Again:
      thisData = None
      break
    msg.append(log.Event.from_bytes(thisData))
  if len(msg) > 0:
    #print str(len(msg))
    
    for a in msg:
       print json.dumps(a.to_dict())
