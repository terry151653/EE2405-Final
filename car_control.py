import curses
from pickletools import StackObject
import time
import sys
import erpc
from bbcar_control import *


def main():

    """
    The curses.wrapper function is an optional function that
    encapsulates a number of lower-level setup and teardown
    functions, and takes a single function to run when
    the initializations have taken place.
    """

    if len(sys.argv) != 2:
        print("Usage: python bbcar_control.py <serial port to use>")
        exit()

    # Initialize all erpc infrastructure
    global client
    global myDistance
    global mySpeed
    global myStatus
    myDistance = erpc.Reference()
    mySpeed = erpc.Reference()
    myStatus = erpc.Reference()
    xport = erpc.transport.SerialTransport(sys.argv[1], 9600)
    client_mgr = erpc.client.ClientManager(xport, erpc.basic_codec.BasicCodec)
    client = client.BBCarServiceClient(client_mgr)
    
    
    curses.wrapper(bbcar_control)


def curses_main(w):

    """
    This function is called curses_main to emphasise that it is
    the logical if not actual main function, called by curses.wrapper.
    """

    w.addstr("----------------------------------\n")
    #w.addstr("| Use arrow keys to control car. |\n")
    w.addstr("|          %f                   |\n", )
    w.addstr("---------------------------------\n")
    w.refresh()

    bbcar_control(w)


def bbcar_control(w):
    w.nodelay(True)
    while True:
        char = w.getch()
        w.move(5, 0)
        if char == ord('q'): break  # q
        elif char == ord('d'):
            w.clrtobot()
            w.addstr("Distance.")
            w.refresh()
            client.returndistance(myDistance)
            w.addstr(str(myDistance.value))
        elif char == ord('s'):
            w.clrtobot()
            w.addstr("Speed.")
            w.refresh()
            client.returnspeed(mySpeed)
            w.addstr(str(mySpeed.value))
        elif char == ord('n'):
            w.clrtobot()
            w.addstr("Status.")
            w.refresh()
            client.returnstatus(myStatus)
            w.addstr(str(myStatus.value))
        else: pass
        time.sleep(0.1)

main()