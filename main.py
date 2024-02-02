# TxRxLOphaseMeasurements Project
# main.py file
# Developed with python 3.9
# V0: 12-2023 - initial version
# V1: 01-2024 - add ITER sweep, tailor the pa_gains per rx_gain setting, add comment in setup regarding the Jlink
#               serial number in the swd.py file. Add flag 'write_data_file' to function
#               adc_capture_and_post_process_1rx()  to allow option to choose if thethe data file for each ADC capture
#               and filtered result is written.
# V1.1: 02-01-2024 - broke code into functional blocks and allow for user input.
# For use with in6xx E0 or F0
# Objective: In ZIF mode, to characterize the phase relationship between the TX LO and RX LO under different conditions.
# This script is to replace the manual process of GUI setup/configuration/ADC capture, and post-process with a different
# tool ie Octave/Matlab with an automated process.
# Independent variables: channel, PA gain, PA2 enable/disable, RX gain states, RF impedance (antenna, cable), etc.
# Setup: IN6xx E/F DK board connected to PC with Jlink and powered on. Run this script on the same PC.
#        Open swd.py and enter the serial number for your Jlink, around line 42.
# Outputs:
#   ./data/[dir_name]/[filename_cond]_in602_ADC_dump.txt : ADC data dump, same file as if produced by ADC capture button
#       on GUI
#   ./data/[dir_name]/[filename_cond]_ADC_out_filtered.txt : ADC output after CIC and LPF filtering, complex
#   ./data/[dir_name]/[dir_name]_sweep_data.txt : independent variable, magnitude, and phase of IQ vector
#   ./data/[dir_name]/[dir_name]_sweep_data.png : graph of the IQ magnitude and phase over the independent variable

import swd
import pylink
import test_trx
from constants import *

def global_reset_chip():
    swd.WR_WORD(AON_REG_AON_MISC_CTRL, 0x50)
    swd.WR_WORD(AON_REG_AON_GLOBAL_RESET_CTRL, 0)

def apply_rf_ams_reg_optmization(ver):
    # v12 2023.12.19
    # E0/F0 DCDC optimized settings for ripple and efficiency
    # 1.1 pmu_doopd_reg_1to4
    swd.aon_write32(0x44110090, 0x55142051)
    # pmu_doopd_reg_910
    #swd.aon_write32(0x44110098, 0x52a)
    # pmu_doopd_reg_189_pd1_active
    #swd.aon_write32(0x441100a0, 0x2a5051)
    # 4.x pmu_doopd_reg_189_tx
    swd.aon_write32(0x441100a4, 0x2a5543)
    # 5.2 pmu_doopd_reg_189_rx
    swd.aon_write32(0x441100a8, 0x2a5551)
    # 6.x E0/F0 XO settings for EVB XTAL
    swd.WR_WORD(0x44110258, 0x22209066)  # xo_stage_1_reg
    # 8.x AGC related
    # rx_agc_ctrl0
    swd.WR_WORD(0x46a01060, 0x81208090)
    # rx_agc_ctrl1
    swd.WR_WORD(0x46a01064, 0x449209)
    # rx_agc_ctrl2
    swd.WR_WORD(0x46a01068, 0x110411)
    # trx_pll_trig_mux_ctrl
    swd.WR_WORD(0x46a031ec, 0x5111)
    # agc_gain_lut_9
    swd.WR_WORD(0x46a01264, 0x9000)
    # agc_reg
    swd.WR_WORD(0x46a03100, 0x870b02)
    if ver != '12':
        # v13 2024.01.23
        # 9.0 use_hold_orig=1 for repeat locking issue
        swd.WR_WORD(0x46a03090, 0x5b841790)
    if ver != '13':
        # v14 2024.01.31
        # 9.1 ccal_off_sel=7 for repeat locking issue
        swd.WR_WORD(0x46a03094, 0x801efaba)


# Start the main part of the script here
id = input("DUT identifier?\n"
           "0 - NJ56-0\n"
           "1 - NJ56-1\n"
           "2 - NJ48-012\n"
           "3 - COB#1\n"
           "-1 - enter name\n")

if id == '0':
    dut_id = 'NJ56-0'
elif id == '1':
    dut_id = 'NJ56-1'
elif id == '2':
    dut_id = 'NJ48-12'
elif id == '3':
    dut_id = 'COB#1'
elif id == '-1':
    dut_id = input("Enter DUT ID: ")
else:
    print("Not valid option")
    exit()

if dut_id == 'NJ56-0' or dut_id == 'NJ56-1':
    hw = 'IN628E_QFN56_EVB'
    hw_ver = 'V2.0_2023-08-21'
elif dut_id == 'NJ48-12':
    hw = 'IN6xx_DK'
    hw_ver = 'F0-08012023'
elif dut_id == 'COB#1':
    hw = 'unknown'
    hw_ver = 'unknown'
else:
    hw_ver = input("Hardware version (Default = V2.0_2023-08-21):\n").strip() or 'V2.0_2023-08-21'
    hw = input("Hardware (Default = IN628E_QFN56_EVB):\n").strip() or 'IN628E_QFN56_EVB'

rf_reg_opt_ver = input("RF Register Optimization Version (12, 13, or 14) (Default = 14):\n").strip() or '14'

rf_sel = input("TRX (RF) termination (Default = 0)?\n"
           "0 - antenna\n"
           "1 - open\n"
           "2 - cable to DUT\n"
           "3 - cable to antenna\n"
           "-1 - enter name\n").strip() or '0'

if rf_sel == '0':
    rf_termination = 'antenna'
elif rf_sel == '1':
    rf_termination = 'open'
elif rf_sel == '2':
    rf_termination = 'cable_to_dut'
elif rf_sel == '3':
    rf_termination = 'cable_to_antenna'
elif rf_sel == '-1':
    rf_termination = input("Enter RF termination:\n")
else:
    print("Not valid option. Exiting.")
    exit()

test_metadata = {
    "hw": hw,
    "hw_ver": hw_ver,
    "rf_reg_opt_ver": rf_reg_opt_ver,
    "dut_id": dut_id,
    "rf_termination": rf_termination,
}

param_sel = input("Parameter to sweep (Default = 0)?\n"
                  "0 - Channel frequency\n"
                  "1 - PA gain\n"
                  "2 - RX gain\n"
                  "3 - Iterate at 1 PA gain and RX gain\n").strip() or '0'

rx_gain_code = 0
if param_sel == '0':
    param_to_sweep = 'channel'
    channel_from = int(input("Enter starting frequency [MHz]: (Default = 2402)\n").strip() or "2402")
    channel_to = int(input("Enter last frequency [MHz]: (Default = 2480)\n").strip() or "2480")
    param_range = range(channel_from, channel_to + 1)
    rx_gain_code = int(input("Enter RX Gain [0:max - 9:min]: (Default = 0)\n").strip() or '0')
    test_metadata.update({'rx_gain_code': rx_gain_code})
elif param_sel == '1':
    param_to_sweep = 'pa_gain'
    param_range = [0x0, 0x1, 0x8, 0x10, 0x18, 0x20, 0x28, 0x30, 0x40, 0x50, 0x60, 0x78]
    rx_gain_code = int(input("Enter RX Gain [0:max - 9:min]: (Default = 0)\n").strip() or '0')
    test_metadata.update({'rx_gain_code': rx_gain_code})
    ch_MHz = input('Enter channel in MHz: (Default = 2440)\n').strip() or '2440'
    test_metadata.update({'ch_MHz': ch_MHz})
elif param_sel == '2':
    param_to_sweep = 'rx_gain_code'
    param_range = [9, 8, 7, 6, 5, 4, 3, 2, 1, 0]
    ch_MHz = input('Enter channel in MHz: (Default = 2440)\n').strip() or '2440'
    test_metadata.update({'ch_MHz': ch_MHz})
elif param_sel == '3':
    param_to_sweep = 'iter'
    n_iter = int(input("Enter number of iterations: (Default = 100)\n").strip() or "100")
    param_range = range(n_iter)
    rx_gain_code = int(input("Enter RX Gain [0:max - 9:min]: (Default = 0)\n").strip() or '0')
    test_metadata.update({'rx_gain_code': rx_gain_code})
    ch_MHz = input('Enter channel in MHz: (Default = 2440)\n').strip() or '2440'
    test_metadata.update({'ch_MHz': ch_MHz})
else:
    print("Not valid option. Exiting.")
    exit()

pa2_en = bool(input("pa2_en = [0: false, 1: true] (Default = 0)\n").strip() or 0)

if pa2_en is False:
    pa_gains = [0x28, 0x28, 0x30, 0x30, 0x30, 0x40, 0x40, 0x40, 0x40, 0x78]
else:
    pa_gains = [0x18, 0x20, 0x28, 0x28, 0x28, 0x38, 0x38, 0x38, 0x38, 0x78]

if param_sel != '1':
    pa_gain_sel = input('Choose PA gain method [0: constant, 1: depends on RX gain] (Default = 0)\n').strip() or '0'
    if pa_gain_sel == '1':
        pa_gain = pa_gains[rx_gain_code]
    else:
        pa_gain = int(input('PA gain = (Default = 120)\n').strip() or '120')
    test_metadata.update({'pa_gain_sel': pa_gain_sel})
    test_metadata.update({'pa_gain': pa_gain})

toggle_tx = False
test_metadata.update({'toggle_tx': toggle_tx})
test_metadata.update({'pa_gains': pa_gains})
test_metadata.update({'pa2_en': pa2_en})
test_metadata.update({'param_to_sweep': param_to_sweep})


# need to reset the DUT before applying the RF register optimizations
global_reset_chip()
print("Global RESET the DUT ... ")
swd.jlink.close()
swd.jlink.open(swd.serial_no)
swd.jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
swd.jlink.connect('CORTEX-M4', speed=4000, verbose=True)
print('core_id = ' + str(swd.jlink.core_id()))

apply_rf_ams_reg_optmization(rf_reg_opt_ver)

test_trx.measure_tx_rx_lo_phase_difference_over_parameter(test_metadata, param_range)

swd.jlink.close()