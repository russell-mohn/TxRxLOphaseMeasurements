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
