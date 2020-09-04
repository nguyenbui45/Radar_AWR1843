%
% Copyright (c) 2019 Texas Instruments Incorporated
%
% All rights reserved not granted herein.
% Limited License.
%
% Texas Instruments Incorporated grants a world-wide, royalty-free,
% non-exclusive license under copyrights and patents it now or hereafter
% owns or controls to make, have made, use, import, offer to sell and sell ("Utilize")
% this software subject to the terms herein.  With respect to the foregoing patent
% license, such license is granted  solely to the extent that any such patent is necessary
% to Utilize the software alone.  The patent license shall not apply to any combinations which
% include this software, other than combinations with devices manufactured by or for TI ("TI Devices").
% No hardware patent is licensed hereunder.
%
% Redistributions must preserve existing copyright notices and reproduce this license (including the
% above copyright notice and the disclaimer and (if applicable) source code license limitations below)
% in the documentation and/or other materials provided with the distribution
%
% Redistribution and use in binary form, without modification, are permitted provided that the following
% conditions are met:
%
%             * No reverse engineering, decompilation, or disassembly of this software is permitted with respect to any
%               software provided in binary form.
%             * any redistribution and use are licensed by TI for use only with TI Devices.
%             * Nothing shall obligate TI to provide you with source code for the software licensed and provided to you in object code.
%
% If software source code is provided to you, modification and redistribution of the source code are permitted
% provided that the following conditions are met:
%
%   * any redistribution and use of the source code, including any resulting derivative works, are licensed by
%     TI for use only with TI Devices.
%   * any redistribution and use of any object code compiled from the source code and any resulting derivative
%     works, are licensed by TI for use only with TI Devices.
%
% Neither the name of Texas Instruments Incorporated nor the names of its suppliers may be used to endorse or
% promote products derived from this software without specific prior written permission.
%
% DISCLAIMER.
%
% THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
% BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
% IN NO EVENT SHALL TI AND TI'S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
% OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
% OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.

function [] = vod_vital_signs(comportSnum, comportCliNum, cliCfgFileName, polarPlotMode, decisionByMatlab)
if nargin<4, polarPlotMode = '0'; end
if nargin<5, decisionByMatlab = '0'; end

if(ischar(comportSnum))
    comportSnum         = str2num(comportSnum);
    polarPlotMode       = str2num(polarPlotMode);
    decisionByMatlab    = str2num(decisionByMatlab);
end
loadCfg     = 1;
debugFlag   = 0;

%% Global parameters
global platformType
% global TOTAL_PAYLOAD_SIZE_BYTES
global MAX_NUM_OBJECTSbytevecAcc
global OBJ_STRUCT_SIZE_BYTES
global STATS_SIZE_BYTES
global BYTES_AVAILABLE_FCN_CNT

global EXIT_KEY_PRESSED
global BYTE_VEC_ACC_MAX_SIZE
global bytevecAcc
global bytevecAccLen
global readUartFcnCntr
global BYTES_AVAILABLE_FLAG
global BYTES_AVAILABLE_FCN_CNT

global activeFrameCPULoad
global interFrameCPULoad
global guiCPULoad
global guiProcTime
global loggingEnable
% global fidLog;
global matFileObj %==>>
global log_frameCount
global rangeAzimuth_log
global Params
global zonePwr
global zonePwrdB

global counting_box
global counting_xpos
global counting_enable
global counting_used
global counting_frameCount
global counting_zone
global figure_width
global figure_height

global rollingMax
global rollingAvg
global rollingIdx

rollingMax = [500 500 500];
rollingAvg = 500;
rollingIdx = 1;

platformType = hex2dec('a1642');
MAX_NUM_OBJECTS         = 100;
OBJ_STRUCT_SIZE_BYTES   = 12;
STATS_SIZE_BYTES        = 16;
BYTES_AVAILABLE_FCN_CNT = 32*8;

EXIT_KEY_PRESSED = 0;
% BYTE_VEC_ACC_MAX_SIZE = 2^15; %%==>>
BYTE_VEC_ACC_MAX_SIZE = 2^16; %%==>>
bytevecAcc      = zeros(BYTE_VEC_ACC_MAX_SIZE,1);
bytevecAccLen   = 0;
readUartFcnCntr = 0;
BYTES_AVAILABLE_FLAG = 0;
BYTES_AVAILABLE_FCN_CNT = 32*8;

activeFrameCPULoad = zeros(100,1);
interFrameCPULoad = zeros(100,1);
guiCPULoad = zeros(100,1);
guiProcTime = 0;
loggingEnable = 0;
% fidLog = 0;
matFileObj = [];   %==>>

counting_enable = 0;
counting_used = 0;
counting_frameCount = 0;
counting_zone = [0 0 0 0 0 0];

PLOT_DISPLAY_LENGTH = 128;

MMWDEMO_UART_MSG_DETECTED_POINTS = 1;
MMWDEMO_UART_MSG_RANGE_PROFILE   = 2;
MMWDEMO_UART_MSG_NOISE_PROFILE   = 3;
MMWDEMO_UART_MSG_AZIMUT_STATIC_HEAT_MAP = 4;
MMWDEMO_UART_MSG_RANGE_DOPPLER_HEAT_MAP = 5;
MMWDEMO_UART_MSG_STATS = 6;

MMWDEMO_UART_MSG_OD_DEMO_RANGE_AZIMUT_HEAT_MAP = 8;
MMWDEMO_UART_MSG_OD_DEMO_FEATURE_VECTOR = 9;
MMWDEMO_UART_MSG_OD_DEMO_DECISION = 10;
VS_OUTPUT_HEART_BREATHING_RATES = 11;

%% ==>>
NUM_RANGE_BINS_IN_HEATMAP = 64;
NUM_ANGLE_BINS = 48;
NUM_MAX_FRAMES_LOG = 192;

% Each column saves the rangeAzimuth heatmap matrix for a frame (ordered in Azimuth major)
rangeAzimuth_log = zeros(NUM_RANGE_BINS_IN_HEATMAP*NUM_ANGLE_BINS, NUM_MAX_FRAMES_LOG, 'single'); %==>>

% bytevec_cp_max_len = 2^15;
bytevec_cp_max_len = 2^16; %==>>
bytevec_cp = zeros(bytevec_cp_max_len,1);
bytevec_cp_len = 0;

displayUpdateCntr = 0;

packetNumberPrev = 0;

fprintf('Starting UI for Occupancy Detection Demo ....\n');

%% Setup the main figure
figHnd = figure(1);
clf(figHnd);
set(figHnd,'Name','Texas Instruments - Occupancy Detection Demo Visualization','NumberTitle','off')
warning off MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame
jframe = get(figHnd,'javaframe');
jIcon = javax.swing.ImageIcon('texas_instruments.gif');
jframe.setFigureIcon(jIcon);
set(figHnd, 'MenuBar', 'none');
set(figHnd, 'Color', [0.8 0.8 0.8]);
set(figHnd, 'KeyPressFcn', @myKeyPressFcn)
% set(figHnd,'ResizeFcn',@Resize_clbk);
% pause(0.00001);
% set(jframe,'Maximized',1);
% pause(0.00001);

pos = get(gcf, 'Position');
figure_width  = pos(3);
figure_height = pos(4);

btn3 = uicontrol('Style', 'checkbox', 'String', 'Record',...
        'Units','normalized',...
        'Position', [.55+.15 .01  .15 .03],...
        'Value', 0,...
        'Callback', @checkbox_Logging_Callback);

btn4 = uicontrol('Style', 'checkbox', 'String', 'Count',...
        'Units','normalized',...
        'Position', [.15 .01  .15 .03],...
        'Value', 0,...
        'Callback', @checkbox_Counting_Callback);

%% Read Configuration file
cliCfg = readConfigFile(cliCfgFileName);

%% Parse CLI parameters
Params = parseConfig(cliCfg);
Params.dataPath.numAngleBins = NUM_ANGLE_BINS; %==>>

%==>>
theta = (-NUM_ANGLE_BINS/2:NUM_ANGLE_BINS/2-1)/NUM_ANGLE_BINS*(60/90)*pi;
theta_degree = (-NUM_ANGLE_BINS/2:NUM_ANGLE_BINS/2-1)/NUM_ANGLE_BINS*(60/90)*180;
range = (0:Params.dataPath.numRangeBins-1) * Params.dataPath.rangeIdxToMeters;

%% Zone structure
for z = 1:Params.numZones
    zone(z) = define_zone(range, theta_degree, Params.zoneDef{z});
end

%% Configure Data UART port
sphandle = configureSport(comportSnum);

%% Send Configuration Parameters to XWR16xx
% % remove lines that are currently not supported %==>>
% kk = 1;
% while kk<length(cliCfg)
%     if ~isempty(strfind(cliCfg{kk}, 'meanVector')) || ~isempty(strfind(cliCfg{kk}, 'stdVector'))
%         cliCfg(kk) = [];
%     else
%         kk = kk+1;
%     end
% end

if loadCfg == 1
    sendConfigToTarget(comportCliNum, cliCfg, cliCfgFileName);
end

%%==>>
numZones    = Params.numZones;
winLen      = Params.windowLen;
zonePwr     = zeros(winLen, numZones);  % circular buffer
zonePwrdB   = zeros(winLen, numZones);  % circular buffer

avgPwr      = zeros(1, numZones);
avgPwrdB    = zeros(1, numZones);
pwrRatio    = zeros(1, numZones);
pwrRatiodB  = zeros(1, numZones);

coeffMatrix = Params.coeffMatrix;
meanVector  = Params.meanVector;
stdVector   = Params.stdVector;

frameIdx    = 0;

outPhasePlot1 =  nan(1,PLOT_DISPLAY_LENGTH);outPhasePlot1(1) = 0;
outBreathPlot1 = nan(1,PLOT_DISPLAY_LENGTH);outBreathPlot1(1) = 0;
outHeartPlot1  = nan(1,PLOT_DISPLAY_LENGTH);outHeartPlot1(1) = 0;

outPhasePlot2 =  nan(1,PLOT_DISPLAY_LENGTH);outPhasePlot2(1) = 0;
outBreathPlot2 = nan(1,PLOT_DISPLAY_LENGTH);outBreathPlot2(1) = 0;
outHeartPlot2  = nan(1,PLOT_DISPLAY_LENGTH);outHeartPlot2(1) = 0;

dummy_counter = 1;
fake_handle = figure(2);

%Zone 1 is flipped so it is the right zone
subplot_handle_unwrapped_phase_zone_1 = subplot(3,2,2);
subplot_handle_breathing_zone_1 = subplot(3,2,4);
subplot_handle_heart_zone_1 = subplot(3,2,6);

%Zone 2 is flipped so it is the left zone
subplot_handle_unwrapped_phase_zone_2 = subplot(3,2,1);
subplot_handle_breathing_zone_2 = subplot(3,2,3);
subplot_handle_heart_zone_2 = subplot(3,2,5);

hPhaseUnwrapped1 = plot(subplot_handle_unwrapped_phase_zone_1,NaN,'color','r');
hLineBreathing1 = plot(subplot_handle_breathing_zone_1,NaN,'color','g');
hLineHeartRate1 = plot(subplot_handle_heart_zone_1,NaN,'color','b');

hPhaseUnwrapped2 = plot(subplot_handle_unwrapped_phase_zone_2,NaN,'color','r');
hLineBreathing2 = plot(subplot_handle_breathing_zone_2,NaN,'color','g');
hLineHeartRate2 = plot(subplot_handle_heart_zone_2,NaN,'color','b');

xlim(subplot_handle_unwrapped_phase_zone_1,[1 PLOT_DISPLAY_LENGTH])
xlim(subplot_handle_breathing_zone_1,[1 PLOT_DISPLAY_LENGTH])
xlim(subplot_handle_heart_zone_1,[1 PLOT_DISPLAY_LENGTH])

xlim(subplot_handle_unwrapped_phase_zone_2,[1 PLOT_DISPLAY_LENGTH])
xlim(subplot_handle_breathing_zone_2,[1 PLOT_DISPLAY_LENGTH])
xlim(subplot_handle_heart_zone_2,[1 PLOT_DISPLAY_LENGTH])

breathing_rate_phrase = 'Breathing Rate: ';
heart_rate_phrase = 'Heart Rate: ';

%Plot titles
title(subplot_handle_unwrapped_phase_zone_1, 'Right Zone', 'FontSize', 35)
title(subplot_handle_unwrapped_phase_zone_2, 'Left Zone', 'FontSize', 35)

title(subplot_handle_breathing_zone_1,breathing_rate_phrase, 'FontSize', 30)
title(subplot_handle_breathing_zone_2,breathing_rate_phrase, 'FontSize', 30)

title(subplot_handle_heart_zone_1,heart_rate_phrase, 'FontSize', 30)
title(subplot_handle_heart_zone_2,heart_rate_phrase, 'FontSize', 30)

%Zone 1 Labels
xlabel(subplot_handle_unwrapped_phase_zone_1,'Frame Index')
ylabel(subplot_handle_unwrapped_phase_zone_1,'Displacement (a.u.)')

xlabel(subplot_handle_breathing_zone_1, 'Frame Index')
ylabel(subplot_handle_breathing_zone_1, 'Phase (radians)')

xlabel(subplot_handle_heart_zone_1, 'Frame Index')
ylabel(subplot_handle_heart_zone_1, 'Phase (radians)')

%Zone 2 Labels
xlabel(subplot_handle_unwrapped_phase_zone_2,'Frame Index')
ylabel(subplot_handle_unwrapped_phase_zone_2,'Displacement (a.u.)')

xlabel(subplot_handle_breathing_zone_2, 'Frame Index')
ylabel(subplot_handle_breathing_zone_2, 'Phase (radians)')

xlabel(subplot_handle_heart_zone_2, 'Frame Index')
ylabel(subplot_handle_heart_zone_2, 'Phase (radians)')

%Move the two GUIs
movegui(figure(1),'east')
movegui(figure(2),'west')

%% -------------------- Main Loop ------------------------
while (~EXIT_KEY_PRESSED)

    % Read bytes
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

        %% Read the header
        [Header, byteVecIdx] = getHeader(bytevec_cp, byteVecIdx);

        detObj.numObj = 0;

        %% Read each TLV
        for tlvIdx = 1:Header.numTLVs

            [tlv, byteVecIdx] = getTlv(bytevec_cp, byteVecIdx);

            switch tlv.type

                case MMWDEMO_UART_MSG_OD_DEMO_RANGE_AZIMUT_HEAT_MAP  %==>>
                    if (Params.guiMonitor.rangeAzimuthHeatMap == 32)
                      [rangeAzimuth_vec, byteVecIdx] = getOccupDemoRangeAzimuthHeatMap(bytevec_cp, ...
                                                byteVecIdx, ...
                                                Params.dataPath.numRangeBins, ...
                                                NUM_ANGLE_BINS);  % Params.dataPath.numAngleBins
                    else
                      [rangeAzimuth_vec, byteVecIdx] = getOccupDemoShortHeatMap(bytevec_cp, ...
                                                byteVecIdx, ...
                                                Params.dataPath.numRangeBins, ...
                                                NUM_ANGLE_BINS);  % Params.dataPath.numAngleBins
                    end
                    %%====================================================================
                    rangeAzimuth = reshape(rangeAzimuth_vec, NUM_ANGLE_BINS, Params.dataPath.numRangeBins).';
                    frameIdx     = frameIdx + 1;

                case MMWDEMO_UART_MSG_OD_DEMO_FEATURE_VECTOR  %==>>
                    [featureVector, byteVecIdx] = getOccupDemoFeatureVector(bytevec_cp, byteVecIdx);

                case MMWDEMO_UART_MSG_OD_DEMO_DECISION  %==>>
                    [decisionValue, byteVecIdx] = getOccupDemoDecision(bytevec_cp, byteVecIdx,numZones);

                    if (counting_enable == 1)
                        for zIdx = 1:numZones
                            if (decisionValue(zIdx) == 1)
                                counting_zone(zIdx) = counting_zone(zIdx)+1;
                            end
                        end

                        counting_frameCount = counting_frameCount + 1;
                    end

                    if (counting_used == 1)
                        my_title = sprintf('Counting: %d frames\nzone1 %3d zone4 %3d\nzone2 %3d zone5 %3d\nzone3 %3d zone6 %3d', ...
                                             counting_frameCount, counting_zone(1), counting_zone(4), counting_zone(2), counting_zone(5), counting_zone(3), counting_zone(6));

                        set(counting_box, 'String', my_title, 'Position', [counting_xpos 1 120 75]);
                    end

                    %%====================================================================
                    
                case VS_OUTPUT_HEART_BREATHING_RATES
                    [extractedValue, byteVecIdx] = getVitalSignsDemoHeartBreathingRate(bytevec_cp, byteVecIdx);


                case MMWDEMO_UART_MSG_STATS
                    [StatsInfo, byteVecIdx] = getStatsInfo(bytevec_cp, byteVecIdx);
                    % fprintf('StatsInfo: %d, %d, %d %d \n', StatsInfo.interFrameProcessingTime, StatsInfo.transmitOutputTime, StatsInfo.interFrameProcessingMargin, StatsInfo.interChirpProcessingMargin);
                     displayUpdateCntr = displayUpdateCntr + 1;
                     interFrameCPULoad = [interFrameCPULoad(2:end); StatsInfo.interFrameCPULoad];
                     activeFrameCPULoad = [activeFrameCPULoad(2:end); StatsInfo.activeFrameCPULoad];
                     guiCPULoad = [guiCPULoad(2:end); 100*guiProcTime/Params.frameCfg.framePeriodicity];
                     if displayUpdateCntr == 40
                        UpdateDisplayTable(Params);
                        displayUpdateCntr = 0;
                     end

                otherwise

            end

        end % tlvIdx = 1:Header.numTLVs


        %***** Create the display now that all TLVs are processed *****

        if (Params.guiMonitor.rangeAzimuthHeatMap == 0) % no heatmap was read
            rangeAzimuth = zeros(Params.dataPath.numRangeBins, NUM_ANGLE_BINS);
        end

        if decisionByMatlab
            if (Params.guiMonitor.rangeAzimuthHeatMap == 0) % no heatmap was read
                fprintf('Error: No heatmap read for Matlab decision processing\n')
            end

            %% zone-based occupancy detection algorithm
            [decisionValue, featureVec] = zone_occupDetect(rangeAzimuth, coeffMatrix, meanVector,...
                                                           stdVector, frameIdx, winLen, zone, numZones);
            occ_color = [0.8, 0.2, 1.0]; % purple for Matlab
        else
            if (Params.guiMonitor.decision == 0) % no decision vector was read
                decisionValue = [0, 0, 0, 0, 0, 0];
            end

            occ_color = [1.0, 0.0, 0.0]; % red for AWR1642
        end

        if polarPlotMode
            displayPolarHeatmap(rangeAzimuth, theta, range);

            displayPolarZones(decisionValue, numZones, zone, occ_color);
        else
            displayRectangleHeatmap(rangeAzimuth, theta_degree, range);

            displayRectangleZones(decisionValue, numZones, zone, occ_color);
        end

        if (loggingEnable == 1) && (log_frameCount < 192)
            title(['{\color{red}Recording}: ', num2str(log_frameCount+1,'%03d'), '/192 frame']);
        end
        drawnow;
        
        %Output Variables to plot - zone 1
        outPhase1 = extractedValue(1);
        outHeart1 = extractedValue(2);
        outBreath1 = extractedValue(3);
        outHeartRate1 = extractedValue(4);
        outBreathRate1 = extractedValue(5);
        
        %Output Variables to plot - zone 2
        outPhase2 = extractedValue(6);
        outHeart2 = extractedValue(7);
        outBreath2 = extractedValue(8);
        outHeartRate2 = extractedValue(9);
        outBreathRate2 = extractedValue(10);
        
        plot_counter = mod(dummy_counter,PLOT_DISPLAY_LENGTH) + 1;
        %Clear the plots if window is filled
        if(plot_counter == PLOT_DISPLAY_LENGTH)
            outBreathPlot1 = nan(1,PLOT_DISPLAY_LENGTH);
            outHeartPlot1 = nan(1,PLOT_DISPLAY_LENGTH);
            outPhasePlot1 = nan(1,PLOT_DISPLAY_LENGTH);
            outBreathPlot2 = nan(1,PLOT_DISPLAY_LENGTH);
            outHeartPlot2 = nan(1,PLOT_DISPLAY_LENGTH);
            outPhasePlot2 = nan(1,PLOT_DISPLAY_LENGTH);
        end
        %Plot the breath outputs
        
        %Zone 1
        outBreathPlot1(plot_counter) = outBreath1;
        set(hLineBreathing1,'YData',outBreathPlot1);
        tempData = outBreathPlot1; tempData(isnan(outBreathPlot1)) = [];
        dataLimMax = max(tempData);
        dataLimMin = min(tempData);
        if (dataLimMax < 1)
            dataLimMax = 1;
        end
        if (dataLimMin > -1)
             dataLimMin = -1;
        end
        ylim(subplot_handle_breathing_zone_1,[dataLimMin dataLimMax + 0.1])
        
            %Update Breathing Rate for Zone 1
            if(decisionValue(1)==0)
                out_phrase = strcat(breathing_rate_phrase, num2str(0,'%.0f'));
            else
                out_phrase = strcat(breathing_rate_phrase, num2str(outBreathRate1,'%.0f'));
            end
            title(subplot_handle_breathing_zone_1,out_phrase, 'FontSize', 30)
        
        %Zone 2
        outBreathPlot2(plot_counter) = outBreath2;
        set(hLineBreathing2,'YData',outBreathPlot2);
        tempData = outBreathPlot2; tempData(isnan(outBreathPlot2)) = [];
        dataLimMax = max(tempData);
        dataLimMin = min(tempData);
        if (dataLimMax < 1)
            dataLimMax = 1;
        end
        if (dataLimMin > -1)
             dataLimMin = -1;
        end
        ylim(subplot_handle_breathing_zone_2,[dataLimMin dataLimMax + 0.1])
        
            %Update Breathing Rate for Zone 2
            if(decisionValue(2)==0)
                out_phrase = strcat(breathing_rate_phrase, num2str(0,'%.0f'));
            else
                out_phrase = strcat(breathing_rate_phrase, num2str(outBreathRate2,'%.0f'));
            end
            title(subplot_handle_breathing_zone_2,out_phrase, 'FontSize', 30)

        %Plot the heart outputs
        %Zone 1
        outHeartPlot1(plot_counter) = outHeart1;
        set(hLineHeartRate1,'YData',outHeartPlot1);
            
            %Update Heart Rate for Zone 1
            if(decisionValue(1)==0)
                out_phrase = strcat(heart_rate_phrase, num2str(0,'%.0f'));
            else
                out_phrase = strcat(heart_rate_phrase, num2str(outHeartRate1,'%.0f'));
            end
            title(subplot_handle_heart_zone_1,out_phrase, 'FontSize', 30)
        
        %Zone 2
        outHeartPlot2(plot_counter) = outHeart2;
        set(hLineHeartRate2,'YData',outHeartPlot2);
        
            %Update Heart Rate for Zone 1
            if(decisionValue(2)==0)
                out_phrase = strcat(heart_rate_phrase, num2str(0,'%.0f'));
            else
                out_phrase = strcat(heart_rate_phrase, num2str(outHeartRate2,'%.0f'));
            end
            title(subplot_handle_heart_zone_2,out_phrase, 'FontSize', 30)
        
        %Plot the phase outputs
        %Zone 1
        outPhasePlot1(plot_counter) = outPhase1;
        set(hPhaseUnwrapped1,'YData',outPhasePlot1);
        %Zone 2
        outPhasePlot2(plot_counter) = outPhase2;
        set(hPhaseUnwrapped2,'YData',outPhasePlot2);
        
        dummy_counter = dummy_counter + 1;
        
        %Print out Breathing and Heart Rate Values for Debugging Purposes
        %fprintf('%.4f HR #1: \n',outHeartRate1);
        %fprintf('%.4f BR #1: \n',outBreathRate1);
        %fprintf('%.4f HR #2: \n',outHeartRate2);
        %fprintf('%.4f BR #2: \n',outBreathRate2)
        
		
        %***** Do Logging and packet bookkeeping *****

        byteVecIdx = Header.totalPacketLen;

        if ((Header.frameNumber - packetNumberPrev) ~= 1) && (packetNumberPrev ~= 0)
            fprintf('Error: Packets lost: %d, current frame num = %d \n', (Header.frameNumber - packetNumberPrev - 1), Header.frameNumber)
        end
        packetNumberPrev = Header.frameNumber;

        %% Log detected objects
        if (loggingEnable == 1) && log_frameCount<192

            %==>>
            log_frameCount = log_frameCount + 1;
            rangeAzimuth_log(:,log_frameCount) =  rangeAzimuth_vec;

        end

        %% Plot the TLV

    end  % if(magicOk == 1)


    %% Remove processed data
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

end % while (~EXIT_KEY_PRESSED)

% Close and delete handles before exiting
% close(1); % close figure
fclose(sphandle); %close com port
delete(sphandle);
quit force;

return


function displayPolarHeatmap(rangeAzimuth_2plot, theta, range)
    global rollingMax
    global rollingAvg
    global rollingIdx

    figure(1)

    heatmapMax = max(rangeAzimuth_2plot(:));

    % Create a 3 frame rolling average of the max value
    rollingMax(rollingIdx) = heatmapMax;
    rollingAvg = mean(rollingMax);
    rollingIdx = rollingIdx + 1;
    if (rollingIdx == 4)
      rollingIdx = 1;
    end

%   cLim = [0, Inf];
    if (rollingAvg < 1000)
      cLim = [0, 1000];
    else
      cLim = [0, rollingAvg];
    end

    imagesc_polar2(theta, range, rangeAzimuth_2plot, cLim); hold on
    set(gca,'XDir','reverse');

    xlabel('Azimuth [m]');
    ylabel('Range [m]');
    yLim = [0, range(end)];
    xLim = yLim(2)*sin(max(abs(theta))) * [-1,1];
    ylim(yLim);
    xlim(xLim);
    delta = 0.5;
    set(gca, 'Xtick', [-50:delta:50]);
    set(gca, 'Ytick', [0:delta:100]);
    set(gca,'Color', [0.5 0.5 0.5])
    grid on;
return


function displayRectangleHeatmap(rangeAzimuth, theta_degree, range)
    global rollingMax
    global rollingAvg
    global rollingIdx

    figure(1)

    heatmapMax = max(rangeAzimuth_2plot(:));

    % Create a 3 frame rolling average of the max value
    rollingMax(rollingIdx) = heatmapMax;
    rollingAvg = mean(rollingMax);
    rollingIdx = rollingIdx + 1;
    if (rollingIdx == 4)
      rollingIdx = 1;
    end

%   cLim = [0, Inf];
    if (rollingAvg < 1000)
      cLim = [0, 1000];
    else
      cLim = [0, rollingAvg];
    end

    imagesc(theta_degree, range, rangeAzimuth, cLim);
    set(gca,'YDir','normal')
    xlabel('Azimuth Angle [degree]');
    ylabel('Range [m]');
return


function displayPolarZones(decisionValue, numZones, zone, occ_color)

    hold on;
    for zIdx = 1:numZones
        if decisionValue(zIdx)
            plot(zone(zIdx).boundary.x, zone(zIdx).boundary.y, 'Color', occ_color, 'LineWidth', 2.0);
        else
            plot(zone(zIdx).boundary.x, zone(zIdx).boundary.y, 'Color', [0.5, 0.5, 0.5], 'LineWidth', 0.5);
        end
    end
    hold off
return


function displayRectangleZones(decisionValue, numZones, zone, occ_color)

    hold on;
    for zIdx = 1:numZones
        if decisionValue(zIdx)
            rectangle('Position', zone(zIdx).rect, 'EdgeColor', occ_color, 'LineWidth', 2.0);
        else
            rectangle('Position', zone(zIdx).rect, 'EdgeColor', [0.5, 0.5, 0.5], 'LineWidth', 0.5);
        end
    end
    hold off
return


%------------------------------------------------------------------------------
function cliCfg = readConfigFile(cliCfgFileName)
%% Read Configuration file
cliCfgFileId = fopen(cliCfgFileName, 'r');
if cliCfgFileId == -1
    fprintf('File %s not found!\n', cliCfgFileName);
    return
else
    fprintf('Opening configuration file %s ...\n', cliCfgFileName);
end
cliCfg = [];
tline = fgetl(cliCfgFileId);
k = 1;
while ischar(tline)
    cliCfg{k} = tline;
    tline = fgetl(cliCfgFileId);
    k = k + 1;
end
fclose(cliCfgFileId);

return

%------------------------------------------------------------------------------
function sendConfigToTarget(comportCliNum, cliCfg, cliCfgFileName)
spCliHandle = configureCliPort(comportCliNum);

warning off; %MATLAB:serial:fread:unsuccessfulRead
timeOut = get(spCliHandle,'Timeout');
set(spCliHandle,'Timeout',1);
% tStart = tic;
while 1
    fprintf(spCliHandle, ''); cc = fread(spCliHandle,100);
    cc = strrep(strrep(cc,char(10),''),char(13),'');
    if ~isempty(cc)
        break;
    end
    pause(0.1);
    % toc(tStart);
end
set(spCliHandle,'Timeout', timeOut);
warning on;

% Send CLI configuration to XWR1xxx
fprintf('Sending configuration to XWR1xxx %s ...\n', cliCfgFileName);
for k = 1:length(cliCfg)
    if isempty(strrep(strrep(cliCfg{k},char(9),''),char(32),''))
        continue;
    end
    if strcmp(cliCfg{k}(1),'%')
        continue;
    end
    fprintf(spCliHandle, cliCfg{k});
    fprintf('%s\n', cliCfg{k});
    for kk = 1:3
        cc = fgetl(spCliHandle);
        if strcmp(cc,'Done')
            fprintf('%s\n',cc);
            break;
        elseif ~isempty(strfind(cc, 'not recognized as a CLI command'))
            fprintf('%s\n',cc);
            return;
        elseif ~isempty(strfind(cc, 'Error'))
            fprintf('%s\n',cc);
            return;
        end
    end
    pause(0.2)
end
fclose(spCliHandle);
delete(spCliHandle);

return

%------------------------------------------------------------------------------
function [sphandle] = configureCliPort(comportPnum)
%if ~isempty(instrfind('Type','serial'))
%    disp('Serial port(s) already open. Re-initializing...');
%    delete(instrfind('Type','serial'));  % delete open serial ports.
%end
comportnum_str = ['COM' num2str(comportPnum)]
sphandle = serial(comportnum_str, 'BaudRate', 115200);
set(sphandle, 'Parity', 'none')
set(sphandle, 'Terminator', 'LF')

fopen(sphandle);

return

%------------------------------------------------------------------------------
function [sphandle] = configureSport(comportSnum)
global BYTES_AVAILABLE_FCN_CNT;

if ~isempty(instrfind('Type','serial'))
    disp('Serial port(s) already open. Re-initializing...');
    delete(instrfind('Type','serial'));  % delete open serial ports.
end
comportnum_str=['COM' num2str(comportSnum)]
sphandle = serial(comportnum_str,'BaudRate',921600);
set(sphandle,'InputBufferSize', 2^16);
set(sphandle,'Timeout',10);
set(sphandle,'ErrorFcn',@dispError);
set(sphandle,'BytesAvailableFcnMode','byte');
set(sphandle,'BytesAvailableFcnCount', 2^16+1);%BYTES_AVAILABLE_FCN_CNT);
set(sphandle,'BytesAvailableFcn',@readUartCallbackFcn);
fopen(sphandle);

return

%------------------------------------------------------------------------------
function myKeyPressFcn(hObject, event)
    global EXIT_KEY_PRESSED
    if lower(event.Key) == 'q'
        EXIT_KEY_PRESSED  = 1;
    end

return

%------------------------------------------------------------------------------
function [] = readUartCallbackFcn(obj, event)
global bytevecAcc;
global bytevecAccLen;
global readUartFcnCntr;
global BYTES_AVAILABLE_FLAG
global BYTES_AVAILABLE_FCN_CNT
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

%------------------------------------------------------------------------------
function [Header, idx] = getHeader(bytevec, idx, platformType)
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
    % new for SDK 1.2, for 16xx only:
    Header.subFrameNumber = sum(bytevec(idx+[1:4]) .* word);
    idx = idx + 4;
return

%------------------------------------------------------------------------------
function [tlv, idx] = getTlv(bytevec, idx)
    word = [1 256 65536 16777216]';
    tlv.type = sum(bytevec(idx+(1:4)) .* word);
    idx = idx + 4;
    tlv.length = sum(bytevec(idx+(1:4)) .* word);
    idx = idx + 4;
return

%------------------------------------------------------------------------------
function [rp, idx] = getRangeProfile(bytevec, idx, len)
    rp = bytevec(idx+(1:len));
    idx = idx + len;
    rp=rp(1:2:end)+rp(2:2:end)*256;
return

%------------------------------------------------------------------------------
function [Q, idx] = getAzimuthStaticHeatMap(bytevec, idx, numTxAzimAnt, numRxAnt, numRangeBins, numAngleBins)
    len = numTxAzimAnt * numRxAnt * numRangeBins * 4;
    q = bytevec(idx+(1:len));
    idx = idx + len;
    q = q(1:2:end)+q(2:2:end)*256;
    q(q>32767) = q(q>32767) - 65536;
    q = q(1:2:end)+1j*q(2:2:end);
    q = reshape(q, numTxAzimAnt * numRxAnt, numRangeBins);
    Q = fft(q, numAngleBins);
return

%------------------------------------------------------------------------------
function [rangeDoppler, idx] = getRangeDopplerHeatMap(bytevec, idx, numDopplerBins, numRangeBins)
    len = numDopplerBins * numRangeBins * 2;
    rangeDoppler = bytevec(idx+(1:len));
    idx = idx + len;
    rangeDoppler = rangeDoppler(1:2:end) + rangeDoppler(2:2:end)*256;
    rangeDoppler = reshape(rangeDoppler, numDopplerBins, numRangeBins);
    rangeDoppler = fftshift(rangeDoppler,1);
return

%------------------------------------------------------------------------------
function [rangeAzimuth, idx] = getOccupDemoRangeAzimuthHeatMap(bytevec, idx, numRangeBins, numAngleBins) %==>>
    len = numRangeBins * numAngleBins * 4;
    rangeAzimuth = bytevec(idx+1:idx+len);
    idx = idx + len;

    % group 4 bytes typecase to single
    rangeAzimuth = typecast(uint8(rangeAzimuth), 'single');
return

%------------------------------------------------------------------------------
function [rangeAzimuth, idx] = getOccupDemoShortHeatMap(bytevec, idx, numRangeBins, numAngleBins) %==>>
    len = numRangeBins * numAngleBins * 2;
    rangeAzimuth = bytevec(idx+1:idx+len);
    idx = idx + len;

    % group 2 bytes typecase to single
    rangeAzimuth = typecast(uint8(rangeAzimuth), 'uint16');
return

%------------------------------------------------------------------------------
function [featureVector, idx] = getOccupDemoFeatureVector(bytevec, idx) %==>>
    len = 20; % 5 single values
    featureVector = bytevec(idx+1:idx+len);
    idx = idx + len;

    % group 4 bytes typecase to single
    featureVector = typecast(uint8(featureVector), 'single');

return


%------------------------------------------------------------------------------
function [decisionValue, idx] = getOccupDemoDecision(bytevec, idx, numZones) %==>>
    %len = 6; % uint8_t
    len = numZones;
    decisionValue = bytevec(idx+1:idx+len);
    idx = idx + len;

    % group 4 bytes typecase to single
    % decisionValue = typecast(uint8(decisionValue), 'uint32');

return

%------------------------------------------------------------------------------
function [extractedValue, idx] = getVitalSignsDemoHeartBreathingRate(bytevec, idx) %==>>
    num_of_outputs = 10;
    size_of_float = 4; %32-bit values (4 bytes)
    len = size_of_float * num_of_outputs;
    extractedValue = bytevec(idx+1:idx+len);
    idx = idx + len;

    % group 4 bytes typecase to single
    extractedValue = typecast(uint8(extractedValue), 'single');
    %fprintf("%.2f \n",extractedValue);

return


%------------------------------------------------------------------------------
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

%------------------------------------------------------------------------------
function checkbox_Logging_Callback(hObject, eventdata, handles)
global loggingEnable
% global fidLog;
global matFileObj;
global log_frameCount
global rangeAzimuth_log
global Params

if (get(hObject,'Value') == get(hObject,'Max'))
    loggingEnable = 1;
    fid = fopen('lognum.dat', 'r');
    if fid ~= -1
        logNum = fscanf(fid, '%d');
        fclose(fid);
    else
        logNum = 0;
    end
    logNum = logNum +1;
    fid = fopen('lognum.dat', 'w');
    fprintf(fid, '%d\n', logNum);
    fclose(fid);

    %fidLog = fopen(sprintf('log_%03d.dat',logNum),'w');
    matFileObj = matfile(sprintf('log_%03d.mat',logNum), 'Writable', true);     %==>>
    log_frameCount = 0;

else
    loggingEnable = 0;
    if matFileObj ~= 0
        % fclose(fidLog);
        matFileObj.numFrames = log_frameCount;
        matFileObj.rangeAzimuth_log = rangeAzimuth_log(:, 1:log_frameCount);
        matFileObj.Params = Params;
        clear matFileObj;
    end

end
return


%------------------------------------------------------------------------------
function checkbox_Counting_Callback(hObject, eventdata, handles)
global counting_box
global counting_xpos
global counting_enable
global counting_used
global counting_frameCount
global counting_zone
global figure_width

if (get(hObject,'Value') == get(hObject,'Max'))
    counting_enable = 1;
    counting_frameCount = 0;
    counting_zone = [0 0 0 0 0 0];

    pos = get(gcf, 'Position');
    figure_width  = pos(3);
    counting_xpos = (figure_width - 120)/2;

    if (counting_used == 0)
        counting_box = uicontrol('style', 'text', 'Position', [counting_xpos 1 120 75]);
    end

    counting_used = 1;

else
    counting_enable = 0;

end
return


%------------------------------------------------------------------------------
% Read relevant CLI parameters and store into P structure
function [P] = parseConfig(cliCfg)

% global TOTAL_PAYLOAD_SIZE_BYTES
global MAX_NUM_OBJECTS
global OBJ_STRUCT_SIZE_BYTES
global platformType
global STATS_SIZE_BYTES

    P=[];
    for k = 1:length(cliCfg)
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
            P.guiMonitor.featureVector = str2num(C{2});
            P.guiMonitor.decision = str2num(C{3});
            P.guiMonitor.rangeAzimuthHeatMap = str2num(C{4});
        elseif strcmp(C{1},'zoneDef')
            P.numZones = str2num(C{2});
            cellIdx = 2;
            for z = 1:P.numZones
                P.zoneDef{z} =  [str2num(C{cellIdx+1}), str2num(C{cellIdx+2}), str2num(C{cellIdx+3}), str2num(C{cellIdx+4})];
                cellIdx = cellIdx + 4;
            end
        elseif strcmp(C{1},'coeffMatrixRow')
            pair = str2num(C{2}) + 1;
            rowIdx = str2num(C{3}) + 1;
            P.coeffMatrix(pair, rowIdx, 1:6) =  [str2num(C{4}), str2num(C{5}), str2num(C{6}), str2num(C{7}), str2num(C{8}), str2num(C{9})];

        elseif strcmp(C{1},'meanVector')
            pair = str2num(C{2}) + 1;
            P.meanVector =  [pair, str2num(C{3}), str2num(C{4}), str2num(C{5}), str2num(C{6}), str2num(C{7})];

        elseif strcmp(C{1},'stdVector')
            pair = str2num(C{2}) + 1;
            P.stdVector  =  [pair, str2num(C{3}), str2num(C{4}), str2num(C{5}), str2num(C{6}), str2num(C{7})];
        elseif strcmp(C{1},'oddemoParms')
            P.windowLen  =  str2num(C{2});
            P.diagLoadFactor =  str2num(C{3});
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

return

%------------------------------------------------------------------------------
function [y] = pow2roundup (x)
    y = 1;
    while x > y
        y = y * 2;
    end
return

%------------------------------------------------------------------------------
function imagesc_polar2(theta, rr, im, cLim) %==>>
% Plot imagesc-like plot in polar coordinates using pcolor()

if nargin<4, cLim = []; end

% transform data in polar coordinates to Cartesian coordinates.
YY = rr'*cos(theta);
XX = rr'*sin(theta);

% plot data on top of grid
h = pcolor(XX, YY, im);
shading flat
grid on;
axis equal;

%
if ~isempty(cLim)
    caxis(cLim);
end

return

%------------------------------------------------------------------------------
function zone = define_zone(rgVal, azVal, def) %==>>
% zoneDef: range_start range_length azimuth_start azimuth_length.
% range_start and azimuth_start index starts from zero

zone.def    = def;
zone.rgIdx  = (zone.def(1)+1:zone.def(1)+zone.def(2));
zone.azIdx  = (zone.def(3)+1:zone.def(3)+zone.def(4));
zone.rect   = [azVal(zone.azIdx(1)), rgVal(zone.rgIdx(1)), ...
                azVal(zone.azIdx(end))-azVal(zone.azIdx(1)), rgVal(zone.rgIdx(end))-rgVal(zone.rgIdx(1))];

% generates a set of points for the boundary of a zone
zone.boundary = gen_zonePoints(zone.rect);

return

%------------------------------------------------------------------------------
function zonePoints = gen_zonePoints(zoneRect) %==>>
% generates a set of points for the boundary of a zone
% to be overlayed on the polar-cordinate plot

% params
theta           = (zoneRect(1):zoneRect(1)+zoneRect(3)).' * pi/180;
rhoInner        = zoneRect(2);
rhoOuter        = (zoneRect(2)+zoneRect(4));

% points
pointsInner.x   = rhoInner*sin(theta);
pointsInner.y   = rhoInner*cos(theta);
pointsOuter.x   = rhoOuter*sin(theta);
pointsOuter.y   = rhoOuter*cos(theta);

% output
zonePoints.x    = [pointsInner.x; flipud(pointsOuter.x); pointsInner.x(1)];
zonePoints.y    = [pointsInner.y; flipud(pointsOuter.y); pointsInner.y(1)];

return

%------------------------------------------------------------------------------
function g = sigmoid(z) %==>>
%SIGMOID Compute sigmoid function
%   g = SIGMOID(z) computes the sigmoid of z.

g = 1 ./ (1 + exp(-z));

return

%------------------------------------------------------------------------------
function [occupVec, featureVec] = zone_occupDetect(rangeAzimuth, coeffMatrix, meanVector, ...  %==>>
                                     stdVector, frameIdx, winLen, zone, numZones)
global zonePwr
global zonePwrdB

avgPwr      = zeros(1, numZones);
avgPwrdB    = zeros(1, numZones);

% zone-power in each frame
cirBufferIdx = mod(frameIdx-1, winLen) + 1;

% moving window index: newest sample at the end
winIdx = mod((cirBufferIdx-winLen:cirBufferIdx-1), winLen) + 1;

for zIdx = 1:numZones
    % calculate zone power
    rangeAzimuth_rgAzGated          = rangeAzimuth(zone(zIdx).rgIdx, zone(zIdx).azIdx);
    zonePwr(cirBufferIdx, zIdx)     = mean(rangeAzimuth_rgAzGated(:));
    zonePwrdB(cirBufferIdx, zIdx)   = 10*log10(zonePwr(cirBufferIdx, zIdx));

    % features: moving average of zonePwr in dB
    avgPwr(zIdx)                    = mean(zonePwr(winIdx, zIdx));
    avgPwrdB(zIdx)                  = 10*log10( avgPwr(zIdx) );
end

% features: power ratio in dB
pwrRatio    = avgPwr / sum(avgPwr);
pwrRatiodB  = 10*log10(pwrRatio);

% features: correlation coefficient between pairs
corrCoeff   = corrcoef(zonePwrdB(winIdx, :));
xcorrCoeff  = corrCoeff(1, 2);

% form the feature vector
featureVec  = [avgPwrdB, pwrRatiodB, xcorrCoeff]; % 1 x 5  for two zones

% normalize and add one
featureVec  = (featureVec - meanVector) ./ stdVector;
featureVec_  = [1, featureVec].'; % now column vector

% occupancy detection
prob                = sigmoid(coeffMatrix * featureVec_);
[~, class_predict]  = max(prob);
class_predict       = class_predict - 1;
occupVec            = de2bi(class_predict, numZones);

return
