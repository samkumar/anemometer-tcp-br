#!/usr/bin/env python

import subprocess
import sys
import signal
import socket
import sys
import time

def cleanup():
    command = "pkill wpantund"
    process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
    process.wait()

def check():
    command = "wpanctl get Thread:Leader:Address"
    process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
    out, err = process.communicate()
    errno = process.returncode
    #errno = process.wait()
    print out
    if errno != 0:
        print "error LeaderAddrGet"
        return False
    
    command = "wpanctl get NCP:State"
    process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
    out, err = process.communicate()
    errno = process.returncode
    print out
    if errno != 0:
        print "error getNCPState"
        return False

    if 'NCP:State = "associated"' in out:
        return True
    else:
        print "error currentState"
        return False

def run():
    command = "sudo wpantund -o Config:NCP:SocketPath /dev/ttyAMA0 -o Config:NCP:SocketBaud 115200"
    process = subprocess.Popen(command.split(), stderr=subprocess.PIPE)
    while True:
        line = process.stderr.readline()
        if "Finished initializing NCP" in line:
            print "finished starting wpantund"
            break
        elif "Resetting and trying again..." in line:
            print "failed starting wpantund"
            break

    command = "wpanctl set Network:XPANID DEAD00BEEF00CAFE"
    process = subprocess.Popen(command.split())
    process.wait()

    command = "wpanctl set Network:Key 00112233445566778899AABBCCDDEEFF"
    process = subprocess.Popen(command.split())
    process.wait()

    command = "wpanctl config-gateway -d fd11:22::"
    process = subprocess.Popen(command.split())
    process.wait()

    command = "wpanctl form -T router -c 26 OpenThreadDemo"
    process = subprocess.Popen(command.split())
    process.wait()


def signal_handler(signal, frame):
    print "You pressed Ctrl+C! Cleaning up"
    cleanup()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

PACKET_TOLERANCE_SECONDS = 60

if __name__ == "__main__":
    while True:
        cleanup()
        run()
        s = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
        s.bind(("", 54321))
        s.settimeout(PACKET_TOLERANCE_SECONDS)
        while True:
            try:
                # Receive heartbeat from MCU
                data, _ = s.recvfrom(4096)
                if len(data) != 89:
                    print time.time(), "Heartbeat has unexpected length"
                    break
		print time.time(), "Received heartbeat from MCU"

                # Send heartbeat to MCU
                s.sendto(b"\x02", ("fdde:ad00:beef:0:e9f0:45bc:c507:6f0b", 12345))
            except socket.timeout:
                print "Timed out waiting for heartbeat"
        s.close()
