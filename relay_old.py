from ctypes import CDLL, c_int, POINTER
import os, time



if __name__ == "__main__":
    so_path = os.path.abspath('./')
    print(so_path)
    relay_lib = CDLL(so_path + "/usb_relay_device.so")

    
    init = relay_lib.usb_relay_init()
    device_enum = relay_lib.usb_relay_device_enumerate()
    print(device_enum)
    device = relay_lib.usb_relay_device_open(device_enum)

    # channel = None

    while True:
        s = input("입력: ")
        if s == "o":
            print("open")
            channel = relay_lib.usb_relay_device_open_one_relay_channel(device, c_int(1))
        elif s == "c":
            print("close")
            channel = relay_lib.usb_relay_device_close_one_relay_channel(device, c_int(1))
        elif s == "dc":
            print("device close")
            # if channel != None:
            device_close = relay_lib.usb_relay_device_close(device)
        elif s == "e":
            break
        

    device_close = relay_lib.usb_relay_device_close(device )
    realy_exit = relay_lib.usb_relay_exit()