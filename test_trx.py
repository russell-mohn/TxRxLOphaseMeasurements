# test_trx.py - put code to test the TRX here

import swd
import math
import cmath
import os
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal
from time import sleep
from constants import *


def start_rx(freq):
    # print("start RX")
    swd.WR_WORD(0x1258, swd.RD_WORD(0x1258) | (1 << 15))  # enable XO 64MHz if not
    nof = 2 * freq * 1e6 / 32e6 * (1 << 19)
    nof_hex = int(nof)
    swd.WR_WORD(RFTRX_REG_MPLL_2P4_REG_1TO4, nof_hex)
    val = swd.RD_WORD(RFTRX_REG_TRX_PLL_TRIG_MUX_CTRL)
    swd.WR_WORD(RFTRX_REG_TRX_PLL_TRIG_MUX_CTRL, val & (~RFTRX_REG_TRX_PLL_TRIG_MUX_CTRL_CTL_TRIG_TO_CTRL_MPLL_FREQ))
    swd.WR_WORD(IPMAC_REG_BLE_FORCE_TRX, IPMAC_REG_BLE_FORCE_TRX_CTL_FORCE_BLE_TRX_RX_EN
                | IPMAC_REG_BLE_FORCE_TRX_CTL_FORCE_BLE_TRX)


def stop_rx():
    # print("stop RX")
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
                    & (~FRONTEND_REGS_MISC_CTRL0_CTLQ_RX_DDFS_BYP))


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
    for k in range(addr_start, addr_start + 8192 * 1):
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
    #t = np.linspace(0, len(opCIC_D4_noDC) - 1, len(opCIC_D4_noDC)) / 16

    # u = math.exp(1j * t * 2 * math.pi * 0)  # returns a vector of 1's
    # u = round(u * 2**8 * math.sqrt(2)) / 8  # with a period of 16
    # opDDFS = np.multiply(u, opCIC_D4_noDC)
    opDDFS = (2 ** 8 * math.sqrt(2) / 8) * opCIC_D4_noDC
    opDDFS = fixptc_s13(opDDFS)

    #  Low pass CIC 16 MHz
    #  4- stage CIC downsample by 2
    b = np.array([1, 4, 6, 4, 1])
    opCIC2 = signal.lfilter(b, 1, opDDFS)
    opCIC2_D2 = opCIC2[0:-1:2]
    opCIC2_D2 = fixptc_s13(opCIC2_D2 / 16)

    b_8MHz_1 = np.array(
        [0.0312, 0, 0, 0, 0, -0.0312, -0.0312, -0.0312, -0.0312, -0.0312, 0, 0.0312, 0.0938, 0.1250, 0.1562, 0.1875,
         0.1875, 0.1875, 0.1562, 0.1250, 0.0938, 0.0312, 0, -0.0312, -0.0312, -0.0312, -0.0312, -0.0312, 0, 0, 0, 0,
         0.0312])
    b_8MHz_2 = np.array([-0.0625, -0.0625, 0, 0.1250, 0.2500, 0.3125, 0.2500, 0.1250, 0, -0.0625, -0.0625])
    opLPF1 = signal.lfilter(b_8MHz_1, 1, opCIC2_D2)
    opLPF1 = signal.lfilter(np.array([0.5, 0.5]), 1, opLPF1)
    opLPF1 = fixptc_s13(opLPF1)
    # opLPF2 = signal.lfilter(b_8MHz_2, 1, opLPF1)
    # opLPF2 = fixptc_s13(opLPF2)
    return opLPF1


def fixptc_s13(x):
    x_real = fixptc_s13_real(np.real(x))
    x_imag = fixptc_s13_real(np.imag(x))
    # op = complex(x_real, x_imag)
    op = x_real + 1j * x_imag
    return op


def fixptc_s13_real(x):
    op = np.round(x)
    op = np.clip(x, -(2 ** 12 - 1), 2 ** 12 - 1)
    return op


def adc_capture_and_post_process_1rx(dir_name, rx_gain_code, pa2_en, pa_gain, write_data_file):
    if os.path.isdir('./data/' + dir_name) == False:
        try:
            os.mkdir('./data/' + dir_name)
        except OSError as error:
            print(error)

    filename_cond = dir_name

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
        with open('./data/' + dir_name + '/' + filename_cond + "in602_ADC_dump.txt", "w") as txt_file:
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
        with open('./data/' + dir_name + '/' + filename_cond + "ADC_out_filtered.txt", "w") as txt_file:
            for val in adc_out_filtered:
                txt_file.write(str(val) + '\n')
        txt_file.close()

    # analyze the data: max, min, mean, stdev
    imag_avg = np.average(adc_out_filtered[100:-1].imag)
    real_avg = np.average(adc_out_filtered[100:-1].real)
    #print(imag_avg)
    #print(real_avg)
    return complex(real_avg, imag_avg)

def get_mpll_lock_status():
    return (swd.RD_WORD(0x46a03128) >> 31)


def measure_tx_rx_lo_phase_difference_over_parameter(test_metadata, param_range):
    dut_id = test_metadata['dut_id']
    pa_gains = test_metadata['pa_gains']
    pa2_en = test_metadata['pa2_en']
    rf_termination = test_metadata['rf_termination']
    param_to_sweep = test_metadata['param_to_sweep']
    toggle_tx = test_metadata['toggle_tx']

    if param_to_sweep != 'channel':
        ch_MHz = int(test_metadata['ch_MHz'])

    if param_to_sweep != 'rx_gain_code':
        rx_gain_code = int(test_metadata['rx_gain_code'])

    if param_to_sweep != 'pa_gain':
        pa_gain_sel = test_metadata['pa_gain_sel']  # 1 if depend on RX gain, 0 is constant PA gain
        pa_gain = int(test_metadata['pa_gain'])

    pa_configure()

    # enable ZIF mode
    zif_mode_en = True
    cfg_zif_mode(zif_mode_en)
    prog_pll_rx_freq_table_freq_only(zif_mode_en)

    dir_name = dut_id + '_sweep_' + param_to_sweep + '__'
    dir_name = dir_name + str(pa2_en) + '_' + str(rf_termination)

    if param_to_sweep == 'pa_gain':
        dir_name = dir_name + '_' + str(ch_MHz) + '_' + str(rx_gain_code)
    elif param_to_sweep == 'rx_gain_code':
        dir_name = dir_name + '_' + str(ch_MHz) + '_' + str(pa_gain)
    elif param_to_sweep == 'iter':
        dir_name = dir_name + '_' + str(ch_MHz) + '_' + str(rx_gain_code) + '_' + str(pa_gain)
    elif param_to_sweep == 'channel':
        dir_name = dir_name + '_' + str(rx_gain_code) + '_' + str(pa_gain)
    else:
        print('param_to_sweep not set correctly, exiting')
        exit()

    data = []
    for param in param_range:
        if param_to_sweep == 'pa_gain':
            start_rx(ch_MHz)
            iq_avg = adc_capture_and_post_process_1rx(dir_name, rx_gain_code, pa2_en, param, False)
            stop_rx()
        elif param_to_sweep == 'rx_gain_code':
            start_rx(ch_MHz)
            if pa_gain_sel == '1':
                pa_gain = pa_gains[param]
            iq_avg = adc_capture_and_post_process_1rx(dir_name, param, pa2_en, pa_gain, False)
            stop_rx()
        elif param_to_sweep == 'iter':
            start_rx(ch_MHz)
            set_pa_and_txmpll_en(False)
            set_pa_and_txmpll_en(True)
            iq_avg = adc_capture_and_post_process_1rx(dir_name, rx_gain_code, pa2_en, pa_gain, False)
            stop_rx()
        elif param_to_sweep == 'channel':
            start_rx(param)
            if toggle_tx:
                set_pa_and_txmpll_en(False)
            set_pa_and_txmpll_en(True)
            iq_avg = adc_capture_and_post_process_1rx(dir_name, rx_gain_code, pa2_en, pa_gain, False)
            stop_rx()
        else:
            print('param_to_sweep not set correctly, exiting')
            exit()

        # compute radius,angle for the I+j*Q vector
        r, phi = cmath.polar(iq_avg)
        phi_deg = 180 / math.pi * phi
        data.append([param, r, phi_deg])
        print('param = {}  r = {:.0f}  phi_deg = {:.1f}'.format(param, r, phi_deg))

    data = np.array(data)

    set_pa_and_txmpll_en(False)
    stop_rx()

    swept_var = param_to_sweep

    # write analysis results to a file
    if os.path.isdir('./data/' + dir_name) == False:
        try:
            os.mkdir('./data/' + dir_name)
        except OSError as error:
            print(error)

    with open('./data/' + dir_name + '/' + dir_name + ".txt", "w") as txt_file:
        for key, value in test_metadata.items():
            txt_file.write(key + ": " + str(value) + "\n")
        np.savetxt(txt_file, data, fmt='%i %1.0f %1.1f')
    txt_file.close()

    fig, (ax1, ax2) = plt.subplots(2)
    fig.suptitle(dir_name + ' ' + swept_var)
    ax1.plot(data[:,0], data[:,1], 'ro-')
    ax1.set(ylabel = 'Magnitude []')
    ax1.grid()
    ax2.plot(data[:,0], data[:,2], 'bo-')
    ax2.set(xlabel = swept_var, ylabel = 'Angle [deg]')
    ax2.grid()
    fig.savefig('./data/' + dir_name + './' + dir_name + ".png")
