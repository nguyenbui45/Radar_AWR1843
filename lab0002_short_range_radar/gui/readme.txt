%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %
 %      (C) Copyright 2016 Texas Instruments, Inc.
 %
 %  Redistribution and use in source and binary forms, with or without
 %  modification, are permitted provided that the following conditions
 %  are met:
 %
 %    Redistributions of source code must retain the above copyright
 %    notice, this list of conditions and the following disclaimer.
 %
 %    Redistributions in binary form must reproduce the above copyright
 %    notice, this list of conditions and the following disclaimer in the
 %    documentation and/or other materials provided with the
 %    distribution.
 %
 %    Neither the name of Texas Instruments Incorporated nor the names of
 %    its contributors may be used to endorse or promote products derived
 %    from this software without specific prior written permission.
 %
 %  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 %  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 %  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 %  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 %  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 %  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 %  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 %  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 %  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 %  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 %  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 %
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


# The SRR TI Design

The short range radar TI design uses the AWR1642, which is TI's single chip radar sensor with an integrated DSP, to detect upto 200 objects and track upto 24 objects at distances as high as 80-120m (or ~400 feet), travelling as high as 90kmph (~ 55mph).
The AWR1642 is configured to be a multi-mode radar, meaning that, while it tracks objects at 120m, it can also generate a rich point cloud of objects at 20m, so that both cars at long range, and smaller obstacles at 20m can be detected.

All of the processing occurs on the DSP device, and only the final output is sent to the PC for display.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project onto an AWR1642.

### Prerequisites

Install the mmwave SDK (version 1.1 and its dependencies), and follow instructions given there to compile the basic OOB application. Make sure that you are able to compile the OOB application, flash it onto a AWR1642 EVM, and use the OOB applicatione. Note that if you don't intend to develop (or compile) the application, then, it isn't necessary to install the mmwave SDK. However, flashing the SRR binary is still necessary.

The other major pre-requisite is a matlab installation (2015a or later) for the visualisation. If you don't have a matlab installation, install the matlab runtime engine (version 8.5.1, 32 bit - link http://ssd.mathworks.com/supportfiles/downloads/R2015a/deployment_files/R2015aSP1/installers/win32/MCR_R2015aSP1_win32_installer.exe) and use the executables provided (only for windows).

### Installing

Unzip the contents of the SRR.zip file into the folder 'mmwave_sdk_01_01_00_02\packages\ti\demo\xwr16xx\'.

In order to compile the application, follow all instructions given to compile the OOB application, including the installation of the pre-requiisites, and the setting up the paths. Once that is done, change the directory to 'mmwave_sdk_01_01_00_02\packages\ti\demo\srr\' and run 'gmake all'.

## Deployment

The GUI is srr_visualization.exe.

### UART-based usage.
Follow  instructions as in the OOB to load the binary (srrdemo_16xx.bin) onto the AWR1642 chip. Once the binary has been flashed, switch the board to SOP0, connect the UART cable, start up the GUI. In the configuration screen of the GUI,  select the correct ports (Use the 'device manager' application to see what the corrrect port numbers are), and press OK.

The GUI should then start, and display a standard set of outputs.

#### OS Priority
Set the OS priority of the processor affinity for the srr_visualization to 'high' otherwise the update rate may be intermittent. In order to do so start up the 'Task manager' and select the 'srr_visualization.exe' process and then set it's priority to high.

### Grid mapping.
To use the grid mapping feature of the GUI, connect the PC through an OBD reader to the car. Then run the program 'srr\gui\ObdReader\ObdReaderExecutable\read_obd_serial_port_and_write_to_a_file.exe'.

Usage : read_obd_serial_port_and_write_to_a_file <port num> <obdOutputfname>

The program will connect to the OBD port, and request speed information at 2Hz, and write the speed to a file (obdOutputfname).
In the main GUI, select the 'Perform Grid Mapping' option, select OBD output fname, and provide the orientation of the car's motion w.r.t to the radar plane, and press OK.

