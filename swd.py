# swd.py
# setup the jlink
# define common read/write functions for the 600 E/F

import pylink

def aon_read32(address):
    # this version works for 602E or F (not 602C or 725C)
    if 0x1080 <= address <= 0x12D4:
        addr = address & 0x0fff
        addr = addr + 0x44110000
    elif 0x7000 <= address <= 0x7244:
        addr = address & 0xfff
        addr = addr + 0x44111000
    else:
        addr = address
    reg = jlink.memory_read32(addr, 1)
    return reg[0]


def aon_write32(address, val):
    # this version works for 602E or F (not 602C or 725C)
    if 0x1080 <= address <= 0x12D4:
        addr = address & 0xfff
        addr = addr + 0x44110000
    elif 0x7000 <= address <= 0x7244:
        addr = address & 0xfff
        addr = addr + 0x44111000
    else:
        addr = address
    jlink.memory_write32(address, [val])

def RD_WORD(address):
    val = jlink.memory_read32(address, 1)
    return val[0]

def WR_WORD(address, val):
    jlink.memory_write32(address, [val])


serial_no = 0x0133196c
serial_choice = input("Choose the Jlink serial number: (Default = 0)\n"
                      "0 - 0x0133196c\n"
                      "1 - 0x01c0fe28\n"
                      "-1 - not listed\n").strip() or '0'

if serial_choice == '-1':
    serial_no = input("Enter the Jlink serial number, in hex, without the 0x:\n")
    serial_no = int(serial_no, 16)
elif serial_choice == '0':
    serial_no = 0x0133196c
elif serial_choice == '1':
    serial_no = 0x01c0fe28
else:
    print("swd.py: Invalid Jlink serial number. Exiting.\n")
    exit()

jlink = pylink.JLink()
jlink.open(serial_no)
print(jlink.product_name)
jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
jlink.connect('CORTEX-M4', speed=4000, verbose=True)
print('core_id = ' + str(jlink.core_id()))
