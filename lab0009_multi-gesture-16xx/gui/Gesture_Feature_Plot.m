 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %
 %      (C) Copyright 2019 Texas Instruments, Inc.
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

%%%%%%%%% EXPECTED FORMAT OF DATA BEING RECEIVED %%%%%%%%%%%%%%%%%%%%%%%%%%
% Magic Number : [2 1 4 3 6 5 8 7] : 8 bytes
% Header size : 40 bytes (including header)
% TLV: (single for Gesture feature and decision)
% [4] : TLV Type
% [4] : TLV Length
% [4] : Weighted Doppler Feature
% [4] : Instance Enegry Feature
% [4] : Weighted Range Feature
% [4] : Azimuth Doppler Correlation Feature
% [4] : Weighted Azimuth Freq Feature
% [4] : Weighted Elevation Freq Feature
% [4] : Frame Counter
% [4*7] : Possibility of Gesture (including background)
% [4] : reserved
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%platform    : supported platform: AWR1642ODS
%comportSnum : comport on which visualization data is being sent
function [] = Gesture_Feature_Plot(platform, comportSnum)

fprintf('Starting UI for Gesture Demo ....\n'); 


if(ischar(comportSnum))
    comportSnum=str2num(comportSnum);
end

global platformType
if exist('platform')
    if strfind(platform, 'AWR1642ODS')
         platformType = hex2dec('a1642');
    else
        fprintf('Unknown platform \n');
        return
    end
else
    platformType = hex2dec('a1642');
end

global MAX_NUM_OBJECTS;
global OBJ_STRUCT_SIZE_BYTES ;
global TOTAL_PAYLOAD_SIZE_BYTES;
global GESTURE_PKT_SIZE_BYTES
global NUM_PKTS_TO_COLLECT;

range_depth='2';
range_width='2';
MMWDEMO_OUTPUT_MSG_GESTURE_FEATURES = 7;
MMWDEMO_OUTPUT_MSG_ANN_OP_PROB = 8;


MAX_NUM_OBJECTS = 200;
OBJ_STRUCT_SIZE_BYTES = 12;
NUM_ANGLE_BINS = 64;
global NUM_FEATURES;
global QUEUE_LENGTH;
global FEATURE_LENGTH;
global NUM_OP_GETURE_CLASS;
global GestId_device;
global GestProb_device;
global packet_id_mss;
global numOfSampleRecvd;

debugFlag = 0;
NUM_FEATURES   = 6;
FEATURE_LENGTH = 10;
NUM_PKTS_TO_COLLECT = 8;
% HEDAER_LEN + TLV_TYPE + TLV_LEN + NumFeature *4 Bytes + PossibilityOfGesture *4 + Resrvd %
GESTURE_PKT_SIZE_BYTES = (40+4+4+NUM_FEATURES*4+ (4*7) + 4);
numOfSampleRecvd = 0;
GestId_device = zeros(1, NUM_OP_GETURE_CLASS);
GestProb_device = zeros(1, NUM_OP_GETURE_CLASS);
packet_id_mss = 0;
% (6 gestures + 1 background)
NUM_OP_GETURE_CLASS = NUM_FEATURES + 1;
incr_packetNumberPrev = 0;
QUEUE_LENGTH   = FEATURE_LENGTH * NUM_FEATURES;
global queue_counter; 
global wtrange_queue;
global wtdoppler_queue;
global instenergy_queue;
global maxazfreq_queue;
global maxelfreq_queue;
global azdoppcorr_queue;
global expected_packetid;
global P_hist;
global P_hist_device;
global P_hist_len;
global num_gestures;
global swipe_post_filt_params;
global circle_post_filt_params;
num_gestures = 7; % including background
P_hist_len = 13; % will store the running history of 13 length time sequence outputs from ANN
P_hist     = zeros(P_hist_len, num_gestures);
P_hist_device = zeros(P_hist_len, num_gestures); % this is based on prob coming from device
circle_post_filt_params = [8, 13]; % 8 out of last 13
swipe_post_filt_params  = [5, 7];  % 5 out of last 7
expected_packetid = 0;
wtrange_queue       = zeros(1,QUEUE_LENGTH);
wtdoppler_queue     = zeros(1,QUEUE_LENGTH);
instenergy_queue    = ones(1,QUEUE_LENGTH);
maxazfreq_queue     = zeros(1,QUEUE_LENGTH);
maxelfreq_queue     = zeros(1,QUEUE_LENGTH);
azdoppcorr_queue    = zeros(1,QUEUE_LENGTH);
queue_counter = 0;


global STATS_SIZE_BYTES
STATS_SIZE_BYTES = 16;

global readUartFcnCntr;
readUartFcnCntr = 0;

global EXIT_KEY_PRESSED
EXIT_KEY_PRESSED = 0;

global Params
global hndChirpParamTable
hndChirpParamTable = 0;

global BYTE_VEC_ACC_MAX_SIZE
BYTE_VEC_ACC_MAX_SIZE = 2^15;
global bytevecAcc
bytevecAcc = zeros(BYTE_VEC_ACC_MAX_SIZE,1);
global bytevecAccLen
bytevecAccLen = 0;


global BYTES_AVAILABLE_FLAG
BYTES_AVAILABLE_FLAG = 0;

warning off MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame  
%Setup the main figure
figHnd = figure;
pause(0.00001);
frame_h = get(handle(gcf),'JavaFrame');
set(frame_h,'Maximized',1);

clf(figHnd);
if platformType == hex2dec('a1642')
    set(figHnd,'Name','Texas Instruments - xWR16xx mmWave Gesture Visualization','NumberTitle','off')
else
    set(figHnd,'Name','Texas Instruments - Unknown platform mmWave Gesture Visualization','NumberTitle','off')
end

jframe=get(figHnd,'javaframe');
jIcon=javax.swing.ImageIcon('texas_instruments.gif');
jframe.setFigureIcon(jIcon);
set(figHnd, 'Color', [0.8 0.8 0.8]);
set(figHnd, 'KeyPressFcn', @myKeyPressFcn)

% Get the default parameters which is being set internally to DSS
% of Gesture demo application.
DefaultParams();

warning off MATLAB:griddata:DuplicateDataPoints

global StatsInfo
StatsInfo.interFrameProcessingTime = 0;
StatsInfo.transmitOutputTime = 0;
StatsInfo.interFrameProcessingMargin = 0;
StatsInfo.interChirpProcessingMargin = 0;
StatsInfo.interFrameCPULoad = 0;
StatsInfo.activeFrameCPULoad = 0;


global activeFrameCPULoad
global interFrameCPULoad
activeFrameCPULoad = zeros(100,1);
interFrameCPULoad = zeros(100,1);
global guiCPULoad
guiCPULoad = zeros(100,1);


global guiProcTime
guiProcTime = 0;

log2ToLog10 = 20*log10(2);

%Configure monitoring UART port 
sphandle = configureSport(comportSnum);

byteVecIdx = 0;
tStart = tic;
tIdleStart = tic;
timeout_ctr = 0;
bytevec_cp_max_len = 2^15;
bytevec_cp = zeros(bytevec_cp_max_len,1);
bytevec_cp_len = 0;

packetNumberPrev = 0;

global loggingEnable
loggingEnable = 0;
global fidLog;
fidLog = 0;
%Initalize figures
figIdx = 2;

% Not supported in this version 
Gesture_classification_en = 0;

% plot the gesture features
Params.guiMonitor.sendgesturefeatures = 1;
if(Params.guiMonitor.sendgesturefeatures == 1)
    Params.guiMonitor.WtRangeFigHnd     = subplot(4,2,1);
    hold on; xlabel('Sample Number'); ylabel('meters'); title('Weighted Range'); ylim([0 0.5]);
    Params.guiMonitor.WtRangeFigHnd = plot(0,0);
    Params.guiMonitor.WtDoppFigHnd      = subplot(4,2,2);
    hold on; xlabel('Sample Number'); ylabel('m/s'); title('Weighted Doppler'); ylim([-3 3]);
    Params.guiMonitor.WtDoppFigHnd = plot(0,0);
    Params.guiMonitor.InstEnergyFigHnd  = subplot(4,2,3);
    hold on; xlabel('Sample Number'); ylabel('(abs)'); title('Inst. energy'); 
    Params.guiMonitor.InstEnergyFigHnd  = plot(0,0);
    Params.guiMonitor.MazAzfreqFigHnd   = subplot(4,2,4);
    hold on; xlabel('Sample Number'); ylabel('bin'); title('Max Az Freq'); ylim([-32,32]);
    Params.guiMonitor.MazAzfreqFigHnd  = plot(0,0);
    Params.guiMonitor.MazElfreqFigHnd   = subplot(4,2,5);
    hold on; xlabel('Sample Number'); ylabel('bin'); title('Max El Freq'); ylim([-32,32]);
    Params.guiMonitor.MazElfreqFigHnd   = plot(0,0);
    Params.guiMonitor.AzdoppcorrFigHnd  = subplot(4,2,6);
    hold on; xlabel('Sample Number'); ylabel('bin-m/s'); title('Az-Dopp correlation'); ylim([-1 1]);
    Params.guiMonitor.AzdoppcorrFigHnd  = plot(0,0);
    Params.guiMonitor.ANNOpHnd_device = subplot(4,2,[7, 8]);
    
    if(Gesture_classification_en == 1)
        Params.guiMonitor.ANNOpHnd_matlab = subplot(4,2,8);
        %Params.guiMonitor.ANNOpHnd_matlab = subplot(4,2,[7, 8]);     
    end
end


magicNotOkCntr=0;

%-------------------- Main Loop ------------------------
while (~EXIT_KEY_PRESSED)
    %Read bytes
    readUartCallbackFcn(sphandle, 0);
    
    if BYTES_AVAILABLE_FLAG == 1
        BYTES_AVAILABLE_FLAG = 0;
        %fprintf('bytevec_cp_len, bytevecAccLen = %d %d \n',bytevec_cp_len, bytevecAccLen)
        if (bytevec_cp_len + bytevecAccLen) < bytevec_cp_max_len
            bytevec_cp(bytevec_cp_len+1:bytevec_cp_len + bytevecAccLen) = bytevecAcc(1:bytevecAccLen);
            bytevec_cp_len = bytevec_cp_len + bytevecAccLen;
            bytevecAccLen = 0;
        else
            fprintf('Error: Buffer overflow, bytevec_cp_len, bytevecAccLen = %d %d \n',bytevec_cp_len, bytevecAccLen)
        end
    end
    
    bytevecStr = char(bytevec_cp);
    magicOk = 0;
    startIdx = strfind(bytevecStr', char([2 1 4 3 6 5 8 7]));
    if ~isempty(startIdx)
        if startIdx(1) > 1
            bytevec_cp(1: bytevec_cp_len-(startIdx(1)-1)) = bytevec_cp(startIdx(1):bytevec_cp_len);
            bytevec_cp_len = bytevec_cp_len - (startIdx(1)-1);
        end
        if bytevec_cp_len < 0
            fprintf('Error: %d %d \n',bytevec_cp_len, bytevecAccLen)
            bytevec_cp_len = 0;
        end

        totalPacketLen = sum(bytevec_cp(8+4+[1:4]) .* [1 256 65536 16777216]');                
        if bytevec_cp_len >= totalPacketLen
            magicOk = 1;
        else
            magicOk = 0;
        end
    end

    byteVecIdx = 0;
    if(magicOk == 1)
        %fprintf('OK, bytevec_cp_len = %d\n',bytevec_cp_len);
        if debugFlag
            fprintf('Frame Interval = %.3f sec,  ', toc(tStart));
        end
        tStart = tic;
        
        [Header, byteVecIdx] = getHeader(bytevec_cp, byteVecIdx);
        detObj.numObj = 0;
        for tlvIdx = 1:Header.numTLVs
            [tlv, byteVecIdx] = getTlv(bytevec_cp, byteVecIdx);
            switch tlv.type
                                     
                case MMWDEMO_OUTPUT_MSG_GESTURE_FEATURES
                    PlotGesturefeatures(bytevec_cp, byteVecIdx);
                    queue_counter = queue_counter + 1;
                    incr_packetNumberPrev = incr_packetNumberPrev + 1;
                    
                case MMWDEMO_OUTPUT_MSG_ANN_OP_PROB % Not supported in this version
                    PlotGestureANN_Prob(bytevec_cp, byteVecIdx);
                    incr_packetNumberPrev = incr_packetNumberPrev + 1;
                    
                otherwise
            end
        end

        byteVecIdx = Header.totalPacketLen;
        

        if ((Header.frameNumber - packetNumberPrev) ~= 1) && (packetNumberPrev ~= 0)
            fprintf('Error: Packets lost: %d, current frame num = %d \n', (Header.frameNumber - packetNumberPrev - 1), Header.frameNumber)
        end
        packetNumberPrev = Header.frameNumber;

        %Log detected objects
        if (loggingEnable == 1)
            if (Header.numDetectedObj > 0)
                fprintf(fidLog, '%d %d\n', Header.frameNumber, detObj.numObj);
                fprintf(fidLog, '%.3f ', detObj.x);
                fprintf(fidLog, '\n');
                fprintf(fidLog, '%.3f ', detObj.y);
                fprintf(fidLog, '\n');
                if Params.dataPath.numTxElevAnt == 1
                    fprintf(fidLog, '%.3f ', detObj.z);
                    fprintf(fidLog, '\n');
                end
                fprintf(fidLog, '%.3f ', detObj.doppler);
                fprintf(fidLog, '\n');
            end
        end            
  
        guiProcTime = round(toc(tStart) * 1e3);
        if debugFlag
            fprintf('processing time %f secs \n',toc(tStart));
        end
        
        %%%%%%%% Plot Gesture features %%%%%%%%%%%%%%%%
        if(Params.guiMonitor.sendgesturefeatures == 1)
            % do post filtering on gesture probabilities coming from device
            P_hist_device(1:end-1,:) = P_hist_device(2:end,:); P_hist_device(end,:) = GestProb_device; %GestId_device; 
            if(sum(P_hist_device(end - swipe_post_filt_params(2) + 1 : end, 2)) >= swipe_post_filt_params(1)) % check for Right->Left
                GestId = [0 1 0 0 0 0 0];
                
            elseif(sum(P_hist_device(end - swipe_post_filt_params(2) + 1 : end, 3)) >= swipe_post_filt_params(1)) % check for Left->Right
                GestId = [0 0 1 0 0 0 0];
                
            elseif(sum(P_hist_device(end - swipe_post_filt_params(2) + 1 : end, 4)) >= swipe_post_filt_params(1)) % check for Up->Down
                GestId = [0 0 0 1 0 0 0];
                
            elseif(sum(P_hist_device(end - swipe_post_filt_params(2) + 1 : end, 5)) >= swipe_post_filt_params(1)) % check for Down->Up
                GestId = [0 0 0 0 1 0 0];
                
            elseif(sum(P_hist_device(end - circle_post_filt_params(2) + 1 : end, 6)) >= circle_post_filt_params(1)) % check for finger clockwise
                GestId = [0 0 0 0 0 1 0];
                
            elseif(sum(P_hist_device(end - circle_post_filt_params(2) + 1 : end, 7)) >= circle_post_filt_params(1)) % check for finger clockwise
                GestId = [0 0 0 0 0 0 1];
                
            else
                GestId = [1 0 0 0 0 0 0];
                
            end
            
            set(Params.guiMonitor.WtRangeFigHnd, 'Xdata', [1:QUEUE_LENGTH], 'Ydata', wtrange_queue * Params.dataPath.rangeIdxToMeters, 'Linewidth', 2);
            set(Params.guiMonitor.WtDoppFigHnd, 'Xdata', [1:QUEUE_LENGTH], 'Ydata', wtdoppler_queue * Params.dataPath.dopplerResolutionMps, 'Linewidth', 2);
            set(Params.guiMonitor.InstEnergyFigHnd, 'Xdata', [1:QUEUE_LENGTH], 'Ydata', instenergy_queue, 'Linewidth', 2);
            set(Params.guiMonitor.MazAzfreqFigHnd, 'Xdata', [1:QUEUE_LENGTH], 'Ydata', maxazfreq_queue, 'Linewidth', 2);
            set(Params.guiMonitor.MazElfreqFigHnd, 'Xdata', [1:QUEUE_LENGTH], 'Ydata', maxelfreq_queue, 'Linewidth', 2);
            set(Params.guiMonitor.AzdoppcorrFigHnd, 'Xdata', [1:QUEUE_LENGTH], 'Ydata', azdoppcorr_queue, 'Linewidth', 2);
        end
    else
        magicNotOkCntr = magicNotOkCntr + 1;
        %fprintf('Magic word not found! cntr = %d\n', magicNotOkCntr);
    end

    %Remove processed data
    if byteVecIdx > 0
        shiftSize = byteVecIdx;
        bytevec_cp(1: bytevec_cp_len-shiftSize) = bytevec_cp(shiftSize+1:bytevec_cp_len);
        bytevec_cp_len = bytevec_cp_len - shiftSize;
        if bytevec_cp_len < 0
            fprintf('Error: bytevec_cp_len < bytevecAccLen, %d %d \n', bytevec_cp_len, bytevecAccLen)
            bytevec_cp_len = 0;
        end
    end
    if bytevec_cp_len > (bytevec_cp_max_len * 7/8)
        bytevec_cp_len = 0;
    end

    tIdleStart = tic;

    pause(0.01);

    if(toc(tIdleStart) > 2*Params.frameCfg.framePeriodicity/1000)
        timeout_ctr=timeout_ctr+1;
        if debugFlag == 1
            fprintf('Timeout counter = %d\n', timeout_ctr);
        end
        tIdleStart = tic;
    end
end
%close and delete handles before exiting
close(1); % close figure
fclose(sphandle); %close com port
delete(sphandle);
return

function [] = readUartCallbackFcn(obj, event)
global bytevecAcc;
global bytevecAccLen;
global readUartFcnCntr;
global BYTES_AVAILABLE_FLAG
global BYTE_VEC_ACC_MAX_SIZE

bytesToRead = get(obj,'BytesAvailable');
if(bytesToRead == 0)
    return;
end

[bytevec, byteCount] = fread(obj, bytesToRead, 'uint8');

if bytevecAccLen + length(bytevec) < BYTE_VEC_ACC_MAX_SIZE * 3/4
    bytevecAcc(bytevecAccLen+1:bytevecAccLen+byteCount) = bytevec;
    bytevecAccLen = bytevecAccLen + byteCount;
else
    bytevecAccLen = 0;
end

readUartFcnCntr = readUartFcnCntr + 1;
BYTES_AVAILABLE_FLAG = 1;
return

function [] = dispError()
disp('UART error!');
return

function [sphandle] = configureSport(comportSnum)
    global GESTURE_PKT_SIZE_BYTES
    global NUM_PKTS_TO_COLLECT;
    
    comportnum_str = ['COM' num2str(comportSnum)];
    if ~isempty(instrfind('Type','serial'))
        disp('Serial port(s) already open. Re-initializing...');
        delete(instrfind('Type','serial'));  % delete open serial ports.
    end
    sphandle = serial(comportnum_str,'BaudRate',921600);
    set(sphandle,'InputBufferSize', GESTURE_PKT_SIZE_BYTES*NUM_PKTS_TO_COLLECT);
    set(sphandle,'Timeout',5);
    set(sphandle,'ErrorFcn',@dispError);
    set(sphandle,'BytesAvailableFcnMode','byte');
    set(sphandle,'BytesAvailableFcnCount', GESTURE_PKT_SIZE_BYTES*NUM_PKTS_TO_COLLECT);
    set(sphandle,'BytesAvailableFcn',@readUartCallbackFcn);
    fopen(sphandle);
return


function myKeyPressFcn(hObject, event)
    global EXIT_KEY_PRESSED
    if lower(event.Key) == 'q'
        EXIT_KEY_PRESSED  = 1;
        fclose all; % close all file handles
    end

return


function []=plotGrid(R,range_grid_arc)

sect_width=pi/12;  
offset_angle=pi/6:sect_width:5*pi/6;
r=[0:range_grid_arc:R];
%w=linspace(pi/6,5*pi/6,128);
w = linspace(10*pi/180,170*pi/180,128);

for n=2:length(r)
    plot(real(r(n)*exp(1j*w)),imag(r(n)*exp(1j*w)),'color',[0.5 0.5 0.5], 'linestyle', ':')
end


for n=1:length(offset_angle)
    plot(real([0 R]*exp(1j*offset_angle(n))),imag([0 R]*exp(1j*offset_angle(n))),'color',[0.5 0.5 0.5], 'linestyle', ':')
end
return

function [Header, idx] = getHeader(bytevec, idx)
    idx = idx + 8; %Skip magic word
    word = [1 256 65536 16777216]';
    Header.version = sum(bytevec(idx+[1:4]) .* word);
    idx = idx + 4;
    Header.totalPacketLen = sum(bytevec(idx+[1:4]) .* word);
    idx = idx + 4;
    Header.platform = sum(bytevec(idx+[1:4]) .* word);
    idx = idx + 4;
    Header.frameNumber = sum(bytevec(idx+[1:4]) .* word);
    idx = idx + 4;
    Header.timeCpuCycles = sum(bytevec(idx+[1:4]) .* word);
    idx = idx + 4;
    Header.numDetectedObj = sum(bytevec(idx+[1:4]) .* word);
    idx = idx + 4;
    Header.numTLVs = sum(bytevec(idx+[1:4]) .* word);
    idx = idx + 4;
    Header.subframeNumber = sum(bytevec(idx+[1:4]) .* word);
    idx = idx + 4;
return

function [tlv, idx] = getTlv(bytevec, idx)
    word = [1 256 65536 16777216]';
    tlv.type = sum(bytevec(idx+(1:4)) .* word);
    idx = idx + 4;
    tlv.length = sum(bytevec(idx+(1:4)) .* word);
    idx = idx + 4;
return

function [StatsInfo, idx] = getStatsInfo(bytevec, idx)
    word = [1 256 65536 16777216]';
    StatsInfo.interFrameProcessingTime = sum(bytevec(idx+(1:4)) .* word);
    idx = idx + 4;
    StatsInfo.transmitOutputTime = sum(bytevec(idx+(1:4)) .* word);
    idx = idx + 4;
    StatsInfo.interFrameProcessingMargin = sum(bytevec(idx+(1:4)) .* word);
    idx = idx + 4;
    StatsInfo.interChirpProcessingMargin = sum(bytevec(idx+(1:4)) .* word);
    idx = idx + 4;
    StatsInfo.activeFrameCPULoad = sum(bytevec(idx+(1:4)) .* word);
    idx = idx + 4;
    StatsInfo.interFrameCPULoad = sum(bytevec(idx+(1:4)) .* word);
    idx = idx + 4;
return

function PlotGesturefeatures(bytevec_cp, byteVecIdx)
    global queue_counter; 
    global NUM_FEATURES;
    global QUEUE_LENGTH;
    global wtrange_queue;
    global wtdoppler_queue;
    global instenergy_queue;
    global maxazfreq_queue;
    global maxelfreq_queue;
    global azdoppcorr_queue;
    global expected_packetid;
    global NUM_OP_GETURE_CLASS;
    global packet_id_mss;
    global GestId_device;
    global GestProb_device;
    global numOfSampleRecvd;
    global NUM_PKTS_TO_COLLECT;
    
    expected_packetid = expected_packetid + 1;
    charbuffer = bytevec_cp(byteVecIdx+1:byteVecIdx+1+((NUM_FEATURES)*4)+4+32 -1);
    wtdoppler_int  = typecast(uint8(charbuffer(1:4)),'single');
    instenergy_int = typecast(uint8(charbuffer(5:8)),'single');
    wtrange_int    = typecast(uint8(charbuffer(9:12)),'single');
    azdoppcorr_int = typecast(uint8(charbuffer(13:16)),'single');
    maxazfreq_int  = typecast(uint8(charbuffer(17:20)),'single');
    maxelfreq_int  = typecast(uint8(charbuffer(21:24)),'single');
    packet_id  = typecast(uint8(charbuffer(25:28)),'uint32'); %from DSS
    
    GestProb_device(1) = typecast(uint8(charbuffer(29:32)),'single');
    GestProb_device(2) = typecast(uint8(charbuffer(33:36)),'single');
    GestProb_device(3) = typecast(uint8(charbuffer(37:40)),'single');
    GestProb_device(4) = typecast(uint8(charbuffer(41:44)),'single');
    GestProb_device(5) = typecast(uint8(charbuffer(45:48)),'single');
    GestProb_device(6) = typecast(uint8(charbuffer(49:52)),'single');
    GestProb_device(7) = typecast(uint8(charbuffer(53:56)),'single');
    
    
    % check for packet drop
    if(abs(packet_id - expected_packetid) > 1)
        warning(['Gesture Packet No: ' num2str(packet_id) 'dropped ' 'packet_id received: ' num2str(expected_packetid)]);
        expected_packetid = packet_id;
    end
    
    % put samples in queue
    wtrange_queue(1:end-1)    = wtrange_queue(2:end); wtrange_queue(end) = wtrange_int;
    wtdoppler_queue(1:end-1)  = wtdoppler_queue(2:end); wtdoppler_queue(end) = wtdoppler_int;
    instenergy_queue(1:end-1) = instenergy_queue(2:end); instenergy_queue(end) = instenergy_int;
    maxazfreq_queue(1:end-1)  = maxazfreq_queue(2:end); maxazfreq_queue(end) = maxazfreq_int;   
    maxelfreq_queue(1:end-1)  = maxelfreq_queue(2:end); maxelfreq_queue(end) = maxelfreq_int;
    azdoppcorr_queue(1:end-1) = azdoppcorr_queue(2:end); azdoppcorr_queue(end) = azdoppcorr_int;
    
    GestId_device = GestProb_device;
    [~,idx] = max(GestId_device);
    GestId_device = zeros(1, NUM_OP_GETURE_CLASS);
    GestId_device(idx) = 1;
    
    %%- Plot the gesture decision 
    y = (GestId_device);

    %%update this subplot at every 8 samples to hold the display for a moment
    if(numOfSampleRecvd == NUM_PKTS_TO_COLLECT)
        numOfSampleRecvd = 0;
        %h = barv(Params.guiMonitor.ANNOpHnd_device, GestId);
        bar(y), xlabel('Gesture Type'), ylabel('Score'), title('Gesture Decision');   
        set(gca, 'XTickLabel',{'Backgrond', 'Right->Left','Left->Right','Up->Down','Down->Up','Finger-Clockwise','Finger-Anticlockwise'})
    else
        numOfSampleRecvd = numOfSampleRecvd +1;
    end
return

%% Store the default configuration which is alrady stored in the Gesture application
function DefaultParams()
global Params;

Params.dataPath.numTxAzimAnt = 2;
Params.dataPath.numTxElevAnt = 0;
Params.dataPath.numRxAnt = 4;
Params.dataPath.numTxAnt = 2;
Params.dataPath.numChirpsPerFrame = 512;
Params.dataPath.numDopplerBins = 256;
Params.dataPath.numRangeBins = 64;
Params.dataPath.rangeResolutionMeters = 0.0469;
Params.dataPath.rangeIdxToMeters = 0.0469;
Params.dataPath.dopplerResolutionMps = 0.0362;

Params.profileCfg.startFreq = 77;
Params.profileCfg.idleTime = 65;
Params.profileCfg.rampEndTime = 40;
Params.profileCfg.freqSlopeConst = 100;
Params.profileCfg.numAdcSamples = 64;
Params.profileCfg.digOutSampleRate = 2000;

Params.frameCfg.chirpStartIdx = 0;
Params.frameCfg.chirpEndIdx = 3;
Params.frameCfg.numLoops = 128;
Params.frameCfg.numFrames = 0;
Params.frameCfg.framePeriodicity = 60;
return

%Read relevant CLI parameters and store into P structure
function [P] = parseCfg(cliCfg)
global TOTAL_PAYLOAD_SIZE_BYTES
global MAX_NUM_OBJECTS
global OBJ_STRUCT_SIZE_BYTES
global platformType
global STATS_SIZE_BYTES

    P=[];
    for k=1:length(cliCfg)
        C = strsplit(cliCfg{k});
        if strcmp(C{1},'channelCfg')
            P.channelCfg.txChannelEn = str2num(C{3});
            if platformType == hex2dec('a1642')
                P.dataPath.numTxAzimAnt = bitand(bitshift(P.channelCfg.txChannelEn,0),1) +...
                                          bitand(bitshift(P.channelCfg.txChannelEn,-1),1);
                P.dataPath.numTxElevAnt = 0;
            elseif platformType == hex2dec('a1443')
                P.dataPath.numTxAzimAnt = bitand(bitshift(P.channelCfg.txChannelEn,0),1) +...
                                          bitand(bitshift(P.channelCfg.txChannelEn,-2),1);
                P.dataPath.numTxElevAnt = bitand(bitshift(P.channelCfg.txChannelEn,-1),1);
            else
                fprintf('Unknown platform \n');
                return
            end
            P.channelCfg.rxChannelEn = str2num(C{2});
            P.dataPath.numRxAnt = bitand(bitshift(P.channelCfg.rxChannelEn,0),1) +...
                                  bitand(bitshift(P.channelCfg.rxChannelEn,-1),1) +...
                                  bitand(bitshift(P.channelCfg.rxChannelEn,-2),1) +...
                                  bitand(bitshift(P.channelCfg.rxChannelEn,-3),1);
            P.dataPath.numTxAnt = P.dataPath.numTxElevAnt + P.dataPath.numTxAzimAnt;
                                
        elseif strcmp(C{1},'dataFmt')
        elseif strcmp(C{1},'profileCfg')
            P.profileCfg.startFreq = str2num(C{3});
            P.profileCfg.idleTime =  str2num(C{4});
            P.profileCfg.rampEndTime = str2num(C{6});
            P.profileCfg.freqSlopeConst = str2num(C{9});
            P.profileCfg.numAdcSamples = str2num(C{11});
            P.profileCfg.digOutSampleRate = str2num(C{12}); %uints: ksps
        elseif strcmp(C{1},'chirpCfg')
        elseif strcmp(C{1},'frameCfg')
            P.frameCfg.chirpStartIdx = str2num(C{2});
            P.frameCfg.chirpEndIdx = str2num(C{3});
            P.frameCfg.numLoops = str2num(C{4});
            P.frameCfg.numFrames = str2num(C{5});
            P.frameCfg.framePeriodicity = str2num(C{6});
        elseif strcmp(C{1},'guiMonitor')
            P.guiMonitor.something = str2num(C{2});
            P.guiMonitor.detectedObjects = str2num(C{3});
            P.guiMonitor.logMagRange = str2num(C{4});
            P.guiMonitor.noiseProfile = str2num(C{5});
            P.guiMonitor.rangeAzimuthHeatMap = str2num(C{6});
            P.guiMonitor.rangeDopplerHeatMap = str2num(C{7});
            P.guiMonitor.stats = str2num(C{8});
        elseif strcmp(C{1},'GESTURE_FEATURE_CFG')
            P.gesture_feature_cfg.rbin_start = str2num(C{3});
            P.gesture_feature_cfg.rbin_stop  = str2num(C{4});
            P.gesture_feature_cfg.posdoppbin_start = str2num(C{5});
            P.gesture_feature_cfg.posdoppbin_end   = str2num(C{6});
            P.gesture_feature_cfg.negdoppbin_end   = str2num(C{7});
            P.gesture_feature_cfg.negdoppbin_end   = str2num(C{8});
            P.gesture_feature_cfg.detthresh        = str2num(C{9});
        end
        
    end
    P.dataPath.numChirpsPerFrame = (P.frameCfg.chirpEndIdx -...
                                            P.frameCfg.chirpStartIdx + 1) *...
                                            P.frameCfg.numLoops;
    P.dataPath.numDopplerBins = P.dataPath.numChirpsPerFrame / P.dataPath.numTxAnt;
    P.dataPath.numRangeBins = pow2roundup(P.profileCfg.numAdcSamples);
    P.dataPath.rangeResolutionMeters = 3e8 * P.profileCfg.digOutSampleRate * 1e3 /...
                     (2 * P.profileCfg.freqSlopeConst * 1e12 * P.profileCfg.numAdcSamples);
    P.dataPath.rangeIdxToMeters = 3e8 * P.profileCfg.digOutSampleRate * 1e3 /...
                     (2 * P.profileCfg.freqSlopeConst * 1e12 * P.dataPath.numRangeBins);
    P.dataPath.dopplerResolutionMps = 3e8 / (2*P.profileCfg.startFreq*1e9 *...
                                        (P.profileCfg.idleTime + P.profileCfg.rampEndTime) *...
                                        1e-6 * P.dataPath.numDopplerBins * P.dataPath.numTxAnt);
    %Calculate monitoring packet size
    tlSize = 8 %TL size 8 bytes
    TOTAL_PAYLOAD_SIZE_BYTES = 32; % size of header
    P.guiMonitor.numFigures = 1; %One figure for numerical parameers
    if P.guiMonitor.detectedObjects == 1 && P.guiMonitor.rangeDopplerHeatMap == 1
        TOTAL_PAYLOAD_SIZE_BYTES = TOTAL_PAYLOAD_SIZE_BYTES +...
            OBJ_STRUCT_SIZE_BYTES*MAX_NUM_OBJECTS + tlSize;
        P.guiMonitor.numFigures = P.guiMonitor.numFigures + 1; %1 plots: X/Y plot
    end
    if P.guiMonitor.detectedObjects == 1 && P.guiMonitor.rangeDopplerHeatMap ~= 1
        TOTAL_PAYLOAD_SIZE_BYTES = TOTAL_PAYLOAD_SIZE_BYTES +...
            OBJ_STRUCT_SIZE_BYTES*MAX_NUM_OBJECTS + tlSize;
        P.guiMonitor.numFigures = P.guiMonitor.numFigures + 2; %2 plots: X/Y plot and Y/Doppler plot
    end
    if P.guiMonitor.logMagRange == 1
        TOTAL_PAYLOAD_SIZE_BYTES = TOTAL_PAYLOAD_SIZE_BYTES +...
            P.dataPath.numRangeBins * 2 + tlSize;
        P.guiMonitor.numFigures = P.guiMonitor.numFigures + 1;
    end
    if P.guiMonitor.noiseProfile == 1 && P.guiMonitor.logMagRange == 1
        TOTAL_PAYLOAD_SIZE_BYTES = TOTAL_PAYLOAD_SIZE_BYTES +...
            P.dataPath.numRangeBins * 2 + tlSize;
    end
    if P.guiMonitor.rangeAzimuthHeatMap == 1
        TOTAL_PAYLOAD_SIZE_BYTES = TOTAL_PAYLOAD_SIZE_BYTES +...
            (P.dataPath.numTxAzimAnt * P.dataPath.numRxAnt) * P.dataPath.numRangeBins * 4 + tlSize;
        P.guiMonitor.numFigures = P.guiMonitor.numFigures + 1; 
    end
    if P.guiMonitor.rangeDopplerHeatMap == 1
        TOTAL_PAYLOAD_SIZE_BYTES = TOTAL_PAYLOAD_SIZE_BYTES +...
            P.dataPath.numDopplerBins * P.dataPath.numRangeBins * 2 + tlSize;
        P.guiMonitor.numFigures = P.guiMonitor.numFigures + 1;
    end
    if P.guiMonitor.stats == 1
        TOTAL_PAYLOAD_SIZE_BYTES = TOTAL_PAYLOAD_SIZE_BYTES +...
            STATS_SIZE_BYTES + tlSize;
        P.guiMonitor.numFigures = P.guiMonitor.numFigures + 1;
    end
    TOTAL_PAYLOAD_SIZE_BYTES = 32 * floor((TOTAL_PAYLOAD_SIZE_BYTES+31)/32);
    P.guiMonitor.numFigRow = 2;
    P.guiMonitor.numFigCol = ceil(P.guiMonitor.numFigures/P.guiMonitor.numFigRow);
    
return

function [y] = pow2roundup (x)
    y = 1;
    while x > y
        y = y * 2;
    end
return

function PlotGestureANN_Prob(bytevec_cp, byteVecIdx)
    global NUM_OP_GETURE_CLASS;
    global packet_id_mss;
    global GestId_device;
    global GestProb_device;
    global expected_packetid;
    
    charbuffer = bytevec_cp(byteVecIdx+1:byteVecIdx+1+(NUM_OP_GETURE_CLASS+1)*4-1); % 7 geture class IDs (each being 4 bytes) + 1 frameindex (4 bytes)
    GestProb_device(1) = typecast(uint8(charbuffer(1:4)),'single');
    GestProb_device(2) = typecast(uint8(charbuffer(5:8)),'single');
    GestProb_device(3) = typecast(uint8(charbuffer(9:12)),'single');
    GestProb_device(4) = typecast(uint8(charbuffer(13:16)),'single');
    GestProb_device(5) = typecast(uint8(charbuffer(17:20)),'single');
    GestProb_device(6) = typecast(uint8(charbuffer(21:24)),'single');
    GestProb_device(7) = typecast(uint8(charbuffer(25:28)),'single');
    %packet_id_mss = typecast(uint8(charbuffer(29:32)),'single');
    packet_id_mss = typecast(uint8(charbuffer(29:32)),'uint32');
    
    % check for packet drop
    if(abs(packet_id_mss - expected_packetid) > 1)
        warning(['Gesture MSS Device Packet No: ' num2str(packet_id_mss) 'dropped ' 'expected packet id: ' num2str(expected_packetid)]);
    end
    expected_packetid = expected_packetid +1;
    
    GestId_device = GestProb_device;
    [~,idx] = max(GestId_device);
    GestId_device = zeros(1, NUM_OP_GETURE_CLASS);
    GestId_device(idx) = 1;
return
