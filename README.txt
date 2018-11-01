monitorCooling
This code is used to monitor forced liquid cooling of a remote device.  By
monitoring the flowrate of fluid through a device (probably best to monitor on 
return line) and the output and return temperature, it is possible to calculate 
the thermal power as well as warning setpoints.
In this case, it is tailored to monitor a gradient insert for MRI systems, and 
provide audio and visual feedback.

requirements:
Max31856 Adafruit Breakout boards for reading thermocouples
Max31856 Arduino Library (modified for fast read and to accept CS pin input)
E-type thermocouples
Omega FPR-300 (-050 1/2inch NPT) flow meter

c. Trevor Wade Oct 2018
