# TxRxLOphaseMeasurements Project
# main.py file
# Developed with python 3.9
# V0: 12-2023 - initial version
# V1: 01-2024 - add ITER sweep, tailor the pa_gains per rx_gain setting, add comment in setup regarding the Jlink
#               serial number in the swd.py file. Add flag 'write_data_file' to function
#               adc_capture_and_post_process_1rx()  to allow option to choose if thethe data file for each ADC capture
#               and filtered result is written.
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
import math
import cmath
import os
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal
from time import sleep

GLOBAL_REG_BASE: int = 0x44126000
RFTRX_REG_BASE: int = 0x46A03000
IPMAC_REG_BASE: int = 0x46800000
FRONTEND_REGS_BASE: int = 0x46A01000
RFTRX_REG_MPLL_2P4_REG_1TO4: int = RFTRX_REG_BASE + 0x140
RFTRX_REG_TRX_PLL_TRIG_MUX_CTRL: int = RFTRX_REG_BASE + 0x1ec
RFTRX_REG_TRX_PLL_TRIG_MUX_CTRL_CTL_TRIG_TO_CTRL_MPLL_FREQ: int = 0x00000001
RFTRX_REG_PA_MASK_CTRL: int = RFTRX_REG_BASE + 0x240
RFTRX_REG_PA_MASK_CTRL_CTL_TX_PA2_EN: int = 0x4
IPMAC_REG_BLE_FORCE_TRX: int = IPMAC_REG_BASE + 0x610
IPMAC_REG_BLE_FORCE_TRX_CTL_FORCE_BLE_TRX_RX_EN: int = 0x4
IPMAC_REG_BLE_FORCE_TRX_CTL_FORCE_BLE_TRX: int = 0x1
RFTRX_REG_SDADC_REG_1TO4_1M: int = RFTRX_REG_BASE + 0xe0
RFTRX_REG_SDADC_REG_1TO4_2M: int = RFTRX_REG_BASE + 0xe4
FRONTEND_REGS_MISC_CTRL0: int = FRONTEND_REGS_BASE + 0x40
FRONTEND_REGS_MISC_CTRL0_CTLQ_RX_DDFS_BYP: int = 0x80000000
FRONTEND_REGS_RX_AGC_CTRL0: int = FRONTEND_REGS_BASE + 0x60
FRONTEND_REGS_RX_AGC_CTRL2: int = FRONTEND_REGS_BASE + 0x68
FRONTEND_REGS_RX_AGC_CTRL0_CTL_AGC_SRESET: int = 0x40000000


def start_rx(freq):
    #print("start RX")
    swd.WR_WORD(0x1258, swd.RD_WORD(0x1258) | (1 << 15))  # enable XO 64MHz if not
    nof = 2 * freq * 1e6 / 32e6 * (1 << 19)
    nof_hex = int(nof)
    swd.WR_WORD(RFTRX_REG_MPLL_2P4_REG_1TO4, nof_hex)
    val = swd.RD_WORD(RFTRX_REG_TRX_PLL_TRIG_MUX_CTRL)
    swd.WR_WORD(RFTRX_REG_TRX_PLL_TRIG_MUX_CTRL, val & (~RFTRX_REG_TRX_PLL_TRIG_MUX_CTRL_CTL_TRIG_TO_CTRL_MPLL_FREQ))
    swd.WR_WORD(IPMAC_REG_BLE_FORCE_TRX, IPMAC_REG_BLE_FORCE_TRX_CTL_FORCE_BLE_TRX_RX_EN
                | IPMAC_REG_BLE_FORCE_TRX_CTL_FORCE_BLE_TRX)

def stop_rx():
    #print("stop RX")
    val = swd.RD_WORD(RFTRX_REG_TRX_PLL_TRIG_MUX_CTRL)
    swd.WR_WORD(RFTRX_REG_TRX_PLL_TRIG_MUX_CTRL, val | RFTRX_REG_TRX_PLL_TRIG_MUX_CTRL_CTL_TRIG_TO_CTRL_MPLL_FREQ)
    swd.WR_WORD(IPMAC_REG_BLE_FORCE_TRX, 0)


def cfg_zif_mode(enable_zif):
    if enable_zif:
        # Bypass the high pass filter
        swd.WR_WORD(RFTRX_REG_SDADC_REG_1TO4_1M, swd.RD_WORD(RFTRX_REG_SDADC_REG_1TO4_1M) | (1 << 31))
        swd.WR_WORD(RFTRX_REG_SDADC_REG_1TO4_2M, swd.RD_WORD(RFTRX_REG_SDADC_REG_1TO4_2M) | (1 << 31))

        # ADC passband to zero
        swd.WR_WORD(RFTRX_REG_SDADC_REG_1TO4_1M, swd.RD_WORD(RFTRX_REG_SDADC_REG_1TO4_1M) & (~(1 << 5)))
        swd.WR_WORD(RFTRX_REG_SDADC_REG_1TO4_2M, swd.RD_WORD(RFTRX_REG_SDADC_REG_1TO4_2M) & (~(1 << 5)))

        # Bypass the DDFS at digital frontend
        swd.WR_WORD(FRONTEND_REGS_MISC_CTRL0, swd.RD_WORD(FRONTEND_REGS_MISC_CTRL0)
                    | FRONTEND_REGS_MISC_CTRL0_CTLQ_RX_DDFS_BYP)
    else:
        # Enable the high pass filter
        swd.WR_WORD(RFTRX_REG_SDADC_REG_1TO4_1M, swd.RD_WORD(RFTRX_REG_SDADC_REG_1TO4_1M) & (~(1 << 31)))
        swd.WR_WORD(RFTRX_REG_SDADC_REG_1TO4_2M, swd.RD_WORD(RFTRX_REG_SDADC_REG_1TO4_2M) & (~(1 << 31)))

        # ADC passband to intermediate frequency
        swd.WR_WORD(RFTRX_REG_SDADC_REG_1TO4_1M, swd.RD_WORD(RFTRX_REG_SDADC_REG_1TO4_1M) | (1 << 5))
        swd.WR_WORD(RFTRX_REG_SDADC_REG_1TO4_2M, swd.RD_WORD(RFTRX_REG_SDADC_REG_1TO4_2M) | (1 << 5))

        # Enable the DDFS at digital frontend
        swd.WR_WORD(FRONTEND_REGS_MISC_CTRL0, swd.RD_WORD(FRONTEND_REGS_MISC_CTRL0)
                    &(~FRONTEND_REGS_MISC_CTRL0_CTLQ_RX_DDFS_BYP))


def prog_pll_rx_table_zif_freq_only(base_addr):
    word0 = 0x4b10000
    word1 = 0x4b10000
    for ch in range(40):
        swd.WR_WORD(base_addr + 0xc * ch + 0x0, word0)
        swd.WR_WORD(base_addr + 0xc * ch + 0x4, word1)

        word0 = word0 + 0x10000
        word1 = word1 + 0x10000


def prog_pll_rx_table_lif_freq_only(base_addr):
    word0 = 0x4b08000
    word1 = 0x4b00000
    for ch in range(40):
        swd.WR_WORD(base_addr + 0xc * ch + 0x0, word0)
        swd.WR_WORD(base_addr + 0xc * ch + 0x4, word1)

        word0 = word0 + 0x10000
        word1 = word1 + 0x10000


def prog_pll_rx_freq_table_freq_only(enable_zif):
    reg_31ec = swd.RD_WORD(0x46A031EC)
    # disable pll_trig control
    swd.WR_WORD(0x46A031EC, 0)

    # enable CPU to access the PLL trig mem
    swd.WR_WORD(0x46A03180, 0x10)

    if enable_zif:
        prog_pll_rx_table_zif_freq_only(0x46a05000)
    else:
        prog_pll_rx_table_lif_freq_only(0x46a05000)

    # recover
    swd.WR_WORD(0x46A031EC, reg_31ec)
    # enable the whole PLL trig
    swd.WR_WORD(0x46A03180, 0x1)


def force_rf_agc(gain_code_in):
    val = swd.RD_WORD(FRONTEND_REGS_RX_AGC_CTRL0)
    val &= ~0xfff

    swd.WR_WORD(FRONTEND_REGS_RX_AGC_CTRL2, swd.RD_WORD(FRONTEND_REGS_RX_AGC_CTRL2) | (1 << 12))
    val |= gain_code_in | (gain_code_in << 4) | (gain_code_in << 8)
    swd.WR_WORD(FRONTEND_REGS_RX_AGC_CTRL0, val)
    dummy_read_val = swd.RD_WORD(FRONTEND_REGS_RX_AGC_CTRL0)
    swd.WR_WORD(FRONTEND_REGS_RX_AGC_CTRL0, val | FRONTEND_REGS_RX_AGC_CTRL0_CTL_AGC_SRESET)
    dummy_read_val = swd.RD_WORD(FRONTEND_REGS_RX_AGC_CTRL0)
    swd.WR_WORD(FRONTEND_REGS_RX_AGC_CTRL0, val)


def pa_configure():
    swd.WR_WORD(0x46a031ec, swd.RD_WORD(0x46a031ec) & (~(1 << 12)))  # PA gain controlled by register
    swd.WR_WORD(0x46a030d0, swd.RD_WORD(0x46a030d0) | (1 << 11))  # bypass PA binary coding


def set_pa2_en(en):
    if en:
        swd.WR_WORD(RFTRX_REG_PA_MASK_CTRL, swd.RD_WORD(RFTRX_REG_PA_MASK_CTRL) | RFTRX_REG_PA_MASK_CTRL_CTL_TX_PA2_EN)
    else:
        swd.WR_WORD(RFTRX_REG_PA_MASK_CTRL, swd.RD_WORD(RFTRX_REG_PA_MASK_CTRL) & ~RFTRX_REG_PA_MASK_CTRL_CTL_TX_PA2_EN)

def set_pa_and_txmpll_en(enable):
    if enable:
        swd.WR_WORD(0x46a04014, 0xc000c)  # force on PA and MPLL_TX_EN
    else:
        val = swd.RD_WORD(0x46A04014)
        val &= ~0xc000c
        swd.WR_WORD(0x46A04014, val)  # unforce on PA and TX MPLL


def set_pa_en(enable):
    if enable:
        val = swd.RD_WORD(0x46A04014)
        val |= (3 << 2)
        swd.WR_WORD(0x46A04014, val)  # force PA
    else:
        val = swd.RD_WORD(0x46A04014)
        val &= ~(3 << 2)
        swd.WR_WORD(0x46A04014, val)  # force off PA


def set_pa_gain(gain):
    val = swd.RD_WORD(0x46a030d0)
    val &= ~0xff
    val |= gain
    swd.WR_WORD(0x46a030d0, val)
    set_pa_en(False)
    #  sleep(0.5)
    set_pa_en(True)  # the value for the PA gain does not get propagated unless the pa_en is toggled


def adc_capture():
    # Let Mem (96K to 128 K) use by DPU
    for addr in range(0x208000, 0x210000, 4):
        swd.WR_WORD(addr, 0)

    # switch the second half mem for DPU use
    val = swd.RD_WORD(0x44126068)
    val |= (1 << 16)
    swd.WR_WORD(0x44126068, val)
    sleep(0.01)
    swd.WR_WORD(0x46A010E0, 0xfff000)
    sleep(0.01)
    swd.WR_WORD(0x46A010E0, 0xfff001)
    sleep(0.1)
    swd.WR_WORD(0x46A010E0, 0xfff009)

    val = swd.RD_WORD(0x46a010FC)
    addr_start = (val >> 0x8) & 0x3fff
    swd.WR_WORD(0x46A010E0, 0xfff050)

    val = swd.RD_WORD(0x44126068)
    val &= ~(1 << 16)
    swd.WR_WORD(0x44126068, val)

    adc_vals = []
    for k in range(addr_start, addr_start + 8192*1):
        addr = ((k & 0x1fff) << 2) + 0x208000
        val = swd.RD_WORD(addr)
        adc_val = val & 0xffffffff
        adc_vals.append(adc_val)

    return adc_vals


def parse_adc_val(adc_val):
    adc_val = adc_val & 0x0000ffff  # the lowest 4 bytes are the ADC value bit pattern
    adc_out = []
    adc_val_b2_str = "{0:16b}".format(adc_val)
    adc_val_b2_str = adc_val_b2_str.replace(' ', '0')  # convert unsigned int to 16 bits
    # adc_val_b2_str = adc_val_b2_str[::-1]  # flip left-right

    for i in range(16):
        if (i % 2) == 0:
            imag_str = adc_val_b2_str[i]
        else:
            val = complex(int(adc_val_b2_str[i]), int(imag_str))
            adc_out.append(val)
    return adc_out


def parse_adc_vals(adc_vals):
    parsed_adc_vals = []
    for adc_val in adc_vals:
        parsed_adc_vals.append(parse_adc_val(adc_val))
    return parsed_adc_vals


def flatten(xss):
    return [x for xs in xss for x in xs]


def filter_adc_bits(ip):
    b64_to_16 = np.array([1, 4, 10, 20, 31, 40, 44, 40, 31, 20, 10, 4, 1])
    opCIC = signal.lfilter(b64_to_16, 1, ip)
    opCIC_D4 = opCIC[0:-1:4]  # format u9
    dc_comp = complex(128, 128)
    opCIC_D4_noDC = opCIC_D4 - dc_comp
    t = np.linspace(0, len(opCIC_D4_noDC) - 1, len(opCIC_D4_noDC)) / 16

    #u = math.exp(1j * t * 2 * math.pi * 0)  # returns a vector of 1's
    #u = round(u * 2**8 * math.sqrt(2)) / 8  # with a period of 16
    #opDDFS = np.multiply(u, opCIC_D4_noDC)
    opDDFS = (2**8 * math.sqrt(2) / 8)*opCIC_D4_noDC
    opDDFS = fixptc_s13(opDDFS)

    #  Low pass CIC 16 MHz
    #  4- stage CIC downsample by 2
    b = np.array([1, 4, 6, 4, 1])
    opCIC2 = signal.lfilter(b, 1, opDDFS)
    opCIC2_D2 = opCIC2[0:-1:2]
    opCIC2_D2 = fixptc_s13(opCIC2_D2/16)

    b_8MHz_1 = np.array(
        [0.0312, 0, 0, 0, 0, -0.0312, -0.0312, -0.0312, -0.0312, -0.0312, 0, 0.0312, 0.0938, 0.1250, 0.1562, 0.1875,
         0.1875, 0.1875, 0.1562, 0.1250, 0.0938, 0.0312, 0, -0.0312, -0.0312, -0.0312, -0.0312, -0.0312, 0, 0, 0, 0,
         0.0312])
    b_8MHz_2 = np.array([-0.0625, -0.0625, 0, 0.1250, 0.2500, 0.3125, 0.2500, 0.1250, 0, -0.0625, -0.0625])
    opLPF1 = signal.lfilter(b_8MHz_1, 1, opCIC2_D2)
    opLPF1 = signal.lfilter(np.array([0.5, 0.5]), 1, opLPF1)
    opLPF1 = fixptc_s13(opLPF1)
    #opLPF2 = signal.lfilter(b_8MHz_2, 1, opLPF1)
    #opLPF2 = fixptc_s13(opLPF2)
    return opLPF1


def fixptc_s13(x):
    x_real = fixptc_s13_real(np.real(x))
    x_imag = fixptc_s13_real(np.imag(x))
    #op = complex(x_real, x_imag)
    op = x_real + 1j*x_imag
    return op


def fixptc_s13_real(x):
    op = np.round(x)
    op = np.clip(x, -(2**12 - 1), 2**12 - 1)
    return op

def apply_rf_ams_reg_optmization():
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


def adc_capture_and_post_process_1rx(dir_name, rx_gain_code, pa2_en, pa_gain, ch_MHz, write_data_file):
    if os.path.isdir('./data/' + dir_name) == False:
        try:
            os.mkdir('./data/' + dir_name)
        except OSError as error:
            print(error)

    filename_cond = dut_id + '_' + str(ch_MHz) + '_' + str(rx_gain_code) + '_' + str(pa_gain) + '_' + str(pa2_en) + '_'

    # force AGC value
    # 0 is max gain, 9 is min gain
    force_rf_agc(rx_gain_code)

    set_pa2_en(pa2_en)
    set_pa_gain(pa_gain)
    # enable the TX: mpll_tx_en and pa_en
    set_pa_and_txmpll_en(True)

    # read the output of the ADC from the front end
    adc_vals = adc_capture()
    if write_data_file:
        # write the data to a file
        with open('./data/' + dir_name + './' + filename_cond + "in602_ADC_dump.txt", "w") as txt_file:
            for val in adc_vals:
                txt_file.write(str(format(val & 0xffffffff, 'x')) + '\n')
        txt_file.close()

    # parse the coded integer values of the ADC output into its actual bit stream
    # the sigma-delta ADC produces a stream of 0,1's for both I and Q
    adcout = parse_adc_vals(adc_vals)
    adcout = flatten(adcout)  # adcout still looks like a list of lists

    #print(adcout[0:19])
    adc_out_filtered = filter_adc_bits(adcout)

    if write_data_file:
        with open('./data/' + dir_name + './' + filename_cond + "ADC_out_filtered.txt", "w") as txt_file:
            txt_file.write(file_meta_header)
            for val in adc_out_filtered:
                txt_file.write(str(val) + '\n')
        txt_file.close()

    # plot the data?
    #plt.plot(adc_out_filtered.real)
    #plt.plot(adc_out_filtered.imag)
    #plt.show()

    # analyze the data: max, min, mean, stdev
    imag_avg = np.average(adc_out_filtered[100:-1].imag)
    real_avg = np.average(adc_out_filtered[100:-1].real)
    #print(imag_avg)
    #print(real_avg)
    return complex(real_avg, imag_avg)

def get_mpll_lock_status():
    return (swd.RD_WORD(0x46a03128) >> 31)

# Start the main part of the script here

#hw = 'IN6xx_DK'
#hw_ver = 'F0-08012023'
hw = 'IN628E_QFN56_EVB'
hw_ver = 'V2.0_2023-08-21'
#hw_ver = 'V1.0_2022-11-25'
rf_reg_opt_ver = '12'
#dut_id = 'NJ56-0'  # put the device under test ID here
dut_id = 'NJ56-1'  # put the device under test ID here
#dut_id = 'NJ48-012'  # put the device under test ID here
#dut_id = 'QFN56-20'  # E0
#dut_id = 'QFN56-14'  # E0
rf_termination = 'cable_to_dut'  # antenna, open, cable to another DUT

ch_MHz = 2440
rx_gain_code = 8

pa2_en = False
if pa2_en is False:
    #pa_gains = [0x10, 0x18, 0x20, 0x20, 0x20, 0x30, 0x30, 0x30, 0x30, 0x78]
    pa_gains = [0x28, 0x28, 0x30, 0x30, 0x30, 0x40, 0x40, 0x40, 0x40, 0x78]
else:
    #pa_gains = [0x08, 0x10, 0x18, 0x18, 0x18, 0x28, 0x28, 0x28, 0x28, 0x78]
    pa_gains = [0x18, 0x20, 0x28, 0x28, 0x28, 0x38, 0x38, 0x38, 0x38, 0x78]

#pa_gain = pa_gains[rx_gain_code]
pa_gain = 0x78
apply_rf_ams_reg_optmization()
pa_configure()

# enable RX Mode on a particular channel
start_rx(ch_MHz)

# enable ZIF mode
zif_mode_en = True
cfg_zif_mode(zif_mode_en)
prog_pll_rx_freq_table_freq_only(zif_mode_en)

# choose what independent variable to sweep
PA_GAIN_SWEEP = False
RX_GAIN_SWEEP = False
CH_SWEEP = True
TOGGLE_TX = True  # this is only applicable during channel sweeps
NDOTF_SWEEP = False
ITER_SWEEP = False

if PA_GAIN_SWEEP:
    file_meta_header = "hw = {}\nhw_ver = {}\nrf_reg_opt_ver = {}\ndut_id = {}\nrf_termination = {}\nch_MHz = {}\
                            \nrx_gain_code = {}\n\n" \
        .format(hw, hw_ver, rf_reg_opt_ver, dut_id, rf_termination, ch_MHz,
                rx_gain_code)

    dir_name = dut_id + '_' + str(ch_MHz) + '_' + str(rx_gain_code) + '_[X]_' + str(pa2_en) + '_' + str(rf_termination)

elif RX_GAIN_SWEEP:
    file_meta_header = "hw = {}\nhw_ver = {}\nrf_reg_opt_ver = {}\ndut_id = {}\nrf_termination = {}\nch_MHz = {}\
                        \npa_gain = {:x}\npa2_en = {}\n\n" \
                        .format(hw, hw_ver, rf_reg_opt_ver, dut_id, rf_termination, ch_MHz,
                                pa_gain, pa2_en)

    dir_name = dut_id + '_' + str(ch_MHz) + '_[X]_' + str(pa_gain) + '_' + str(pa2_en) + '_' + str(rf_termination)

elif CH_SWEEP:
    file_meta_header = "hw = {}\nhw_ver = {}\nrf_reg_opt_ver = {}\ndut_id = {}\nrf_termination = {}\
                            \nrx_gain_code = {}\npa_gain = {:x}\npa2_en = {}\n\n" \
        .format(hw, hw_ver, rf_reg_opt_ver, dut_id, rf_termination,
                rx_gain_code, pa_gain, pa2_en)

    if TOGGLE_TX:
        dir_name = dut_id + '_[X]_' + str(rx_gain_code) + '_' + str(pa_gain) + '_' + str(pa2_en) + '_' + str(
            rf_termination) + '_toggleTX'
    else:
        dir_name = dut_id + '_[X]_' + str(rx_gain_code) + '_' + str(pa_gain) + '_' + str(pa2_en) + '_' + str(
            rf_termination)
elif NDOTF_SWEEP:
    file_meta_header = "hw = {}\nhw_ver = {}\nrf_reg_opt_ver = {}\ndut_id = {}\nrf_termination = {}\
                                \nrx_gain_code = {}\npa_gain = {:x}\npa2_en = {}\n\n" \
        .format(hw, hw_ver, rf_reg_opt_ver, dut_id, rf_termination,
                rx_gain_code, pa_gain, pa2_en)
    dir_name = dut_id + '_' + str(ch_MHz) + '_[NDOTF_SWEEP]_' + str(rx_gain_code) + '_' + str(pa_gain) + '_' + str(
        pa2_en) + '_' + str(rf_termination)
elif ITER_SWEEP:
    file_meta_header = "hw = {}\nhw_ver = {}\nrf_reg_opt_ver = {}\ndut_id = {}\nrf_termination = {}\
                                    \nrx_gain_code = {}\npa_gain = {:x}\npa2_en = {}\n\n" \
        .format(hw, hw_ver, rf_reg_opt_ver, dut_id, rf_termination,
                rx_gain_code, pa_gain, pa2_en)
    dir_name = dut_id + '_' + str(ch_MHz) + '_[ITER_SWEEP]_' + str(rx_gain_code) + '_' + str(pa_gain) + '_' + str(
        pa2_en) + '_' + str(rf_termination)
else:
    print("No sweep is TRUE")

if NDOTF_SWEEP:
    # val = swd.RD_WORD(0x46a03090)
    # val &= ~(0x3 << 25)
    # val |= (0x2 << 25)  # decrease the lock detect window size from 4ns -> 2ns
    # val &= ~(0x3 << 27)
    # val |= (0x2 << 27)  # decrease the lock detect ncycles from 15 -> 10
    #swd.WR_WORD(0x46a03090, val)
    lock_sts = get_mpll_lock_status()
    print('ch_MHz = {}  lock_sts = {}\n'.format(ch_MHz, lock_sts))
    span = 36  # span in MHz around the initial ch_MHz to sweep the N.f
    data = []
    for freq in range(round(ch_MHz - span/2), round(ch_MHz + span/2)+1):
        nof = 2 * freq * 1e6 / 32e6 * (1 << 19)
        nof_hex = int(nof)
        swd.WR_WORD(RFTRX_REG_MPLL_2P4_REG_1TO4, nof_hex)
        sleep(0.3)
        lock_sts = get_mpll_lock_status()
        iq_avg = adc_capture_and_post_process_1rx(dir_name, rx_gain_code, pa2_en, pa_gain, freq, False)
        r, phi = cmath.polar(iq_avg)
        phi_deg = 180 / math.pi * phi
        data.append([freq, lock_sts, r, phi_deg])
        print('ch_MHz = {}  lock_sts = {}  r = {:.0f}  phi_deg = {:.1f}'.format(freq, lock_sts, r, phi_deg))

    data = np.array(data)

if PA_GAIN_SWEEP:
    #pa_gains = [0x1, 0x4, 0x8, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x78]
    #pa_gains = [0x1, 0x4, 0x8, 0x10, 0x20, 0x30, 0x40]
    pa_gains = range(24, 48, 2)
    #pa_gains = range(0,5)
    #pa_gains = [0x1, 0x78]
    data = []
    for pa_gain in pa_gains:
        iq_avg = adc_capture_and_post_process_1rx(dir_name, rx_gain_code, pa2_en, pa_gain, ch_MHz, False)
        # compute radius,angle for the I+j*Q vector
        r, phi = cmath.polar(iq_avg)
        phi_deg = 180 / math.pi * phi
        data.append([pa_gain, r, phi_deg])
        print('pa_gain = {}  r = {:.0f}  phi_deg = {:.1f}'.format(pa_gain, r, phi_deg))

    data = np.array(data)
# End PA Gain Sweeps -------------------------------------------------------------------------------------------------

if RX_GAIN_SWEEP:
    rx_gain_codes = [9,8,7,6,5,4,3,2,1,0]
    #rx_gain_codes = [9, 8, 7]
    #rx_gain_codes = [8,7,6,5,4,3,2]
    #rx_gain_codes = [1, 0]
    data = []
    for rx_gain_code in rx_gain_codes:
        pa_gain = pa_gains[rx_gain_code]
        iq_avg = adc_capture_and_post_process_1rx(dir_name, rx_gain_code, pa2_en, pa_gain, ch_MHz, False)
        # compute radius,angle for the I+j*Q vector
        r, phi = cmath.polar(iq_avg)
        phi_deg = 180 / math.pi * phi
        data.append([rx_gain_code, r, phi_deg])
        print('rx_gain_code = {}  r = {:.0f}  phi_deg = {:.1f}'.format(rx_gain_code, r, phi_deg))

    data = np.array(data)
# End RX Gain Sweeps -------------------------------------------------------------------------------------------------

if ITER_SWEEP:
    data = []
    for i in range(0, 100):
        start_rx(ch_MHz)
        set_pa_and_txmpll_en(False)
        set_pa_and_txmpll_en(True)
        iq_avg = adc_capture_and_post_process_1rx(dir_name, rx_gain_code, pa2_en, pa_gain, ch_MHz, False)

        # compute radius,angle for the I+j*Q vector
        r, phi = cmath.polar(iq_avg)
        phi_deg = 180 / math.pi * phi
        data.append([i, r, phi_deg])
        print('i = {}  r = {:.0f}  phi_deg = {:.1f}'.format(i, r, phi_deg))
        stop_rx()

    data = np.array(data)
# End Iteration Sweeps ----------------------------------------------------------------------------------------------

if CH_SWEEP:
    chs = range(2402, 2481)
    data = []
    for ch_MHz in chs:
        start_rx(ch_MHz)
        if TOGGLE_TX:
            set_pa_and_txmpll_en(False)
        set_pa_and_txmpll_en(True)

        iq_avg = adc_capture_and_post_process_1rx(dir_name, rx_gain_code, pa2_en, pa_gain, ch_MHz, False)
        # compute radius,angle for the I+j*Q vector
        r, phi = cmath.polar(iq_avg)
        phi_deg = 180 / math.pi * phi
        data.append([ch_MHz, r, phi_deg])
        print('ch_MHz = {}  r = {:.0f}  phi_deg = {:.1f}'.format(ch_MHz, r, phi_deg))
        stop_rx()

    data = np.array(data)
# End Channel Sweeps ----------------------------------------------------------------------------------------------

set_pa_and_txmpll_en(False)
stop_rx()

if PA_GAIN_SWEEP:
    swept_var = 'PA Gain'
elif RX_GAIN_SWEEP:
    swept_var = 'RX Gain'
elif CH_SWEEP:
    swept_var = 'Channel'
elif ITER_SWEEP:
    swept_var = 'Iteration'
elif NDOTF_SWEEP:
    swept_var = 'Ch by N.f'
else:
    print("No swept variable")

if PA_GAIN_SWEEP | RX_GAIN_SWEEP | CH_SWEEP | ITER_SWEEP:
    # write analysis results to a file
    with open('./data/' + dir_name + './' + dir_name + ".txt", "w") as txt_file:
        txt_file.write(file_meta_header)
        np.savetxt(txt_file, data, fmt='%i %1.0f %1.1f')
    txt_file.close()

    fig, (ax1, ax2) = plt.subplots(2)
    fig.suptitle(dir_name + ' ' + swept_var)
    ax1.plot(data[:,0], data[:,1], 'ro-')
    ax1.set(ylabel = 'Magnitude []')
    ax2.plot(data[:,0], data[:,2], 'bo-')
    ax2.set(xlabel = swept_var, ylabel = 'Angle [deg]')
    fig.savefig('./data/' + dir_name + './' + dir_name + ".png")

if NDOTF_SWEEP:
    # write analysis results to a file
    with open('./data/' + dir_name + './' + dir_name + ".txt", "w") as txt_file:
        txt_file.write(file_meta_header)
        np.savetxt(txt_file, data, fmt='%i %i %1.0f %1.1f')
    txt_file.close()

    fig, (ax1, ax2, ax3) = plt.subplots(3)
    fig.suptitle(dir_name + ' ' + swept_var)
    ax1.plot(data[:,0], data[:,1], 'ko-')
    ax1.set(ylabel='Lock Detect []')
    ax2.plot(data[:,0], data[:,2], 'ro-')
    ax2.set(ylabel = 'Magnitude []')
    ax3.plot(data[:,0], data[:,3], 'bo-')
    ax3.set(xlabel = swept_var, ylabel = 'Angle [deg]')
    fig.savefig('./data/' + dir_name + './' + dir_name + ".png")

swd.jlink.close()
