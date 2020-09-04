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
%%%%%%%%%%%EXPECTED FORMAT OF DATA BEING RECEIVED%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Magic Number : [2 1 4 3  6 5  8 7] : 8 bytes
%Inter frame processing time in CPU cycles : 4 bytes
%Noise Energy :  4 bytes (uint32)
%Number of valid objects : 4 bytes
%The following data are sent depending on the cli configuration
%Object Data :  10 bytes per object *MAX_NUM_OBJECTS = 500 bytes [see structure of object data below objOut_t]
%Range Profile : 1DfftSize * sizeof(uint16_t) bytes
%2D range bins at zero Doppler, complex symbols including all received
%virtual antennas : 1DfftSize * sizeof(uint32_t) * numVirtualAntennas
%Doppler-Range 2D FFT log magnitude matrix: 1DfftSize * 2Dfftsize * sizeof(uint16_t)
%
% typedef volatile struct objOut
% {
%     uint8_t   rangeIdx;
%     uint8_t   dopplerIdx ;
%     uint16_t  peakVal;
%     int16_t  x; //in meters
%     int16_t  y; //in meters
%     int16_t  z; //in meters
% } objOut_t;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [] = park_assist_visualizer()

fprintf('Starting Park Assist Visualizer....\n');


if exist('maxRangeProfileYaxis')
    if(ischar(maxRangeProfileYaxis))
        maxRangeProfileYaxis = str2num(maxRangeProfileYaxis);
    end
else
    maxRangeProfileYaxis = 1e6;
end


if exist('debugFlag')
    if strfind(debugFlag, 'debugFlag')
         debugFlag = 1;
    else
        debugFlag = 0;
    end
else
    debugFlag = 0;
end

global platformType
platformType = hex2dec('a1642');

% if ischar(loadCfg)
%     loadCfg = str2num(loadCfg);
% end

global MAX_NUM_OBJECTS;
global OBJ_STRUCT_SIZE_BYTES ;
global CLS_STRUCT_SIZE_BYTES ;
global TOTAL_PAYLOAD_SIZE_BYTES;
global MIN_CLUSTER_SIZE;
global MIN_NORMAL_DIST;

ODSDEMO_UART_MSG_DETECTED_POINTS = 1;
ODSDEMO_UART_MSG_CLUSTERS   = 2;
ODSDEMO_UART_MSG_RANGE_AZIMUT_HEAT_MAP = 3;
ODSDEMO_UART_MSG_RANGE_ELEV_HEAT_MAP = 4;

global cluster;
global lastClustCount;

lastClustCount = 0;

%display('version 0.6');

% below defines correspond to the ODS demo code
MAX_NUM_OBJECTS = 200;
OBJ_STRUCT_SIZE_BYTES = 12;
CLS_STRUCT_SIZE_BYTES = 12;
TOTAL_AZIMUTH_WIDTH = 180;
NUM_ANGLE_BINS = 60;
ANGLE_RESOLUTION = TOTAL_AZIMUTH_WIDTH / NUM_ANGLE_BINS;
MIN_CLUSTER_SIZE = 0.2;
MIN_NORMAL_DIST = 1.0;

global STATS_SIZE_BYTES
STATS_SIZE_BYTES = 16;


global bytevec_log;
bytevec_log = [];

global readUartFcnCntr;
readUartFcnCntr = 0;

global ELEV_VIEW
ELEV_VIEW = 3;
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

global BYTES_AVAILABLE_FCN_CNT
BYTES_AVAILABLE_FCN_CNT = 32*8;


% Execution will pause here until the profile configuration is Loaded.
sensor.mode = 1;
[h hScatter hAxisOccupancy hPatch hText hClust] = plots_occupancy(sensor);

indxHist = 0;
maxHist = 5;
Params = h.params;
chirpParams = h.params.dataPath;
hDataSerialPort = h.DATACOM.hDataSerialPort;
hControlSerialPort = h.UARTCOM.hControlSerialPort;

warning off MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame
%jframe=get(figHnd,'javaframe');
%jIcon=javax.swing.ImageIcon('texas_instruments.gif');
%jframe.setFigureIcon(jIcon);
%set(figHnd, 'MenuBar', 'none');
%set(figHnd, 'Color', [0.8 0.8 0.8]);
%set(figHnd, 'KeyPressFcn', @myKeyPressFcn)
%set(figHnd,'ResizeFcn',@Resize_clbk);

pause(0.00001);

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

displayUpdateCntr =0;


log2ToLog10 = 20*log10(2);
if platformType == hex2dec('a1642')
    log2Qformat = 1/(256*Params.dataPath.numRxAnt*Params.dataPath.numTxAnt);
elseif platformType == hex2dec('a1443') || platformType == hex2dec('a1111')
    if Params.dataPath.numTxAnt == 3
        log2Qformat = (4/3)*1/512;
    else
        log2Qformat = 1/512;
    end
else
    fprintf('Unknown platform\n');
    return
end

global maxRngProfYaxis
global maxRangeProfileYaxisLin
global maxRangeProfileYaxisLog
maxRangeProfileYaxisLin = maxRangeProfileYaxis;
maxRangeProfileYaxisLog = log2(maxRangeProfileYaxis) * log2ToLog10;
global rangeProfileLinearScale
rangeProfileLinearScale = 1;
if rangeProfileLinearScale == 1
    maxRngProfYaxis = maxRangeProfileYaxisLin;
else
    maxRngProfYaxis = maxRangeProfileYaxisLog;
end

global dopRngHeatMapFlag
dopRngHeatMapFlag = 1;


byteVecIdx = 0;
rangePlotYaxisMin = 1e10;
rangePlotXaxisMax = Params.dataPath.rangeIdxToMeters * Params.dataPath.numRangeBins;
tStart = tic;
tIdleStart = tic;
timeout_ctr = 0;
bytevec_cp_max_len = 2^15;
bytevec_cp = zeros(bytevec_cp_max_len,1);
bytevec_cp_len = 0;
packetNumberPrev = 0;


%Initalize figures
figIdx = 2;

if (Params.guiMonitor.rangeAzimuthHeatMap == 1) && ...
   (Params.guiMonitor.rangeElevHeatMap == 1)

  fprintf('ERROR: Cannot display both heatmaps at the same time.\n');
  quit force;
end


if (Params.guiMonitor.clusters == 1) && ...
   (Params.guiMonitor.detectedObjects == 0)

%  fprintf('ERROR: Cannot display both heatmaps at the same time.\n');
%  quit force;
end


%if (Params.guiMonitor.rangeAzimuthHeatMap == 1) || ...
%   (Params.guiMonitor.rangeElevHeatMap == 1)
%    %Range complex bins at zero Doppler all virtual (azimuth) antennas
%    Params.guiMonitor.heatMapFigHnd = subplot(Params.guiMonitor.numFigRow, Params.guiMonitor.numFigCol, figIdx);
%    Params.guiMonitor.heatMapPlotHnd = plot(0,0);
%    figIdx = figIdx + 1;
%    theta = asind([-NUM_ANGLE_BINS/2+1 : NUM_ANGLE_BINS/2-1]'*(2/NUM_ANGLE_BINS));
%    range = [0:Params.dataPath.numRangeBins-1] * Params.dataPath.rangeIdxToMeters;
%    xlin=linspace(-range_width,range_width,200);
%    ylin=linspace(0,range_depth,200);
%    view([0,90])
%    set(Params.guiMonitor.rangeAzimuthHeatMapFigHnd,'Color',[0 0 0.5]);
%    axis('equal')
%    axis(Params.guiMonitor.rangeAzimuthHeatMapFigHnd, [-range_width range_width 0 range_depth])
%    xlabel('Distance along lateral axis (meters)');
%    ylabel('Distance along longitudinal axis (meters)');
%    title('Azimuth-Range Heatmap')
%end

magicNotOkCntr=0;
cluster.numCl = 0;
cluster.numZero = 0;
lastCluster.numCl = 0;

%-------------------- Main Loop ------------------------
while (~EXIT_KEY_PRESSED)
    %Read bytes
    readUartCallbackFcn(hDataSerialPort, 0);

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

    drawClusters = 0;

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
                case ODSDEMO_UART_MSG_DETECTED_POINTS
                    if tlv.length >= OBJ_STRUCT_SIZE_BYTES
                        [detObj, byteVecIdx] = getDetObj(bytevec_cp, ...
                                byteVecIdx, ...
                                tlv.length, ...
                                Params.dataPath.rangeIdxToMeters, ...
                                Params.dataPath.dopplerResolutionMps, ...
                                Params.dataPath.numDopplerBins);
                    end

                case ODSDEMO_UART_MSG_CLUSTERS
                    drawClusters = 1;
                    lastCluster = cluster;
                    [cluster, byteVecIdx] = getClusters(bytevec_cp, ...
                                                byteVecIdx, ...
                                                tlv.length);

                case ODSDEMO_UART_MSG_RANGE_AZIMUT_HEAT_MAP
                    [rangeAzimuth_vec, byteVecIdx] = getHeatMap(bytevec_cp, ...
                                                byteVecIdx, ...
                                                Params.dataPath.numRangeBins, ...
                                                NUM_ANGLE_BINS);

%                    theta = (-NUM_ANGLE_BINS/2:NUM_ANGLE_BINS/2-1)/NUM_ANGLE_BINS*pi;
                    theta_degree = (-NUM_ANGLE_BINS/2:NUM_ANGLE_BINS/2-1)/NUM_ANGLE_BINS*180;
                    range = (0:Params.dataPath.numRangeBins-1) * Params.dataPath.rangeResolutionMeters;
                    rangeAzimuth = reshape(rangeAzimuth_vec, NUM_ANGLE_BINS, Params.dataPath.numRangeBins).';

%                    displayPolarHeatmap(rangeAzimuth, theta, range);
                    displayRectangleHeatmap(rangeAzimuth, theta_degree, range, 1);

                    %%====================================================================

                case ODSDEMO_UART_MSG_RANGE_ELEV_HEAT_MAP
                    [rangeElev_vec, byteVecIdx] = getHeatMap(bytevec_cp, ...
                                                byteVecIdx, ...
                                                Params.dataPath.numRangeBins, ...
                                                NUM_ANGLE_BINS);

%                    theta = (-NUM_ANGLE_BINS/2:NUM_ANGLE_BINS/2-1)/NUM_ANGLE_BINS*pi;
%                    theta_degree = (-NUM_ANGLE_BINS/2:NUM_ANGLE_BINS/2-1)/NUM_ANGLE_BINS*180;
                    az_range = (hAxisOccupancy.XLim(1):hAxisOccupancy.XLim(2));
                    range = (0:Params.dataPath.numRangeBins-1) * Params.dataPath.rangeResolutionMeters;
                    rangeElev = reshape(rangeElev_vec, NUM_ANGLE_BINS, Params.dataPath.numRangeBins).';

%                    displayPolarHeatmap(rangeElev, theta, range);
                    displayRectangleHeatmap(rangeElev, az_range, range, 2);

                otherwise
            end
        end

        byteVecIdx = Header.totalPacketLen;

        if ((Header.frameNumber - packetNumberPrev) ~= 1) && (packetNumberPrev ~= 0)
            fprintf('Error: Packets lost: %d, current frame num = %d \n', (Header.frameNumber - packetNumberPrev - 1), Header.frameNumber)
        end
        packetNumberPrev = Header.frameNumber;

        vizSetting = hAxisOccupancy.UserData;

        if (Params.guiMonitor.detectedObjects == 1)
            set(hText,'String',['Frame: ' num2str(Header.frameNumber) '      Objects: ' num2str(detObj.numObj) '      Clusters: ' num2str(cluster.numCl) '(' num2str(cluster.numZero) ')'])
%            indxHist = indxHist +1;
%            if indxHist > vizSetting.maxHist
                indxHist = 1;
%            end

            if detObj.numObj > 0
                % Display Detected objects
                if Params.dataPath.numTxElevAnt >= 1
                    %Plot detected objects in 3D
                    %view(ELEV_VIEW);

                    % update point locations
                    xLim = hAxisOccupancy.XLim;
                    yLim = hAxisOccupancy.YLim;
                    zLim = hAxisOccupancy.ZLim;
                    if (vizSetting.showDetObj == 1)
                      validInd = (detObj.x<=xLim(2) & detObj.x>=xLim(1) & detObj.y<=yLim(2) & detObj.y>=yLim(1) & detObj.z<=zLim(2) & detObj.z>=zLim(1));
                    else
                      validInd = zeros(detObj.numObj);
                    end

                    if(find(validInd))

                        % Display the detected objects:
                        set(hScatter(indxHist), 'Xdata',detObj.x(validInd), 'Ydata', detObj.y(validInd), 'Zdata',detObj.z(validInd));

                        % update color
                        switch vizSetting.typeMarkerColor
                            case 1
                                %Z
                                set(hScatter(indxHist),'CData', [detObj.z(validInd)]')
                            case 2
                                %Y
                                set(hScatter(indxHist),'CData', [detObj.y(validInd)]')
                            case 3
                                %X
                                set(hScatter(indxHist),'CData', [detObj.x(validInd)]')
                            case 4
                                %Doppler Sign
                                dopplerSign = [((detObj.dopplerIdx(validInd))<0).*1] + [((detObj.dopplerIdx(validInd))==0).*2] + [((detObj.dopplerIdx(validInd))>0).*3];
                                set(hScatter(indxHist),'CData', dopplerSign')
                            case 5
                                %Time
                                if(vizSetting.maxHist > 1)
                                    set(hScatter(indxHist),'CData', repelem(indxHist, sum(validInd))')
                                else
                                    set(hScatter(indxHist),'CData', repmat([0 1 0],sum(validInd),1))
                                end
                            case 6
                                %Constant
                                set(hScatter(indxHist),'CData', repmat([0 1 0],sum(validInd),1))
                        end

                        % update size
                        switch vizSetting.typeMarkerSize
                            case 2
                            % Intensity
                            %disp(detObj.peakVal)
                                set(hScatter(indxHist), 'SizeData',(50*min(ceil(detObj.peakVal(validInd)./100),3)));
                            case 3
                            % Doppler
                                set(hScatter(indxHist), 'SizeData', (abs(detObj.dopplerIdx(validInd))./Params.dataPath.numDopplerBins).*300);
                        end

                    else
                        set(hScatter(indxHist), 'Xdata',0, 'Ydata', 0, 'Zdata', 0, 'CData', [0 0 0], 'SizeData', 25);
                    end
                end
            else
                if Params.dataPath.numTxElevAnt >= 1
                    set(hScatter(indxHist), 'Xdata',0, 'Ydata', 0, 'Zdata', 0, 'CData', [0 0 0], 'SizeData', 25);
                    updateOccupancyGridAll(hAxisOccupancy, hScatter, indxHist, vizSetting.maxHist);
                end
            end
        end

        if ((Params.guiMonitor.clusters == 1) && (drawClusters == 1))
          if (vizSetting.showClusters == 0)
            cluster.numCl = 0;
          end
          drawClusterCubes(hAxisOccupancy, hClust, cluster);
        end

        if (Params.guiMonitor.rangeAzimuthHeatMap == 1) || ...
           (Params.guiMonitor.rangeElevHeatMap == 1)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %Range complex bins at zero Doppler all virtual (azimuth) antennas
            %subplot(Params.guiMonitor.rangeAzimuthHeatMapFigHnd);
            %hold off;

%            theta = asind([-NUM_ANGLE_BINS/2+1 : NUM_ANGLE_BINS/2-1]'*(2/NUM_ANGLE_BINS));
%            range = [0:Params.dataPath.numRangeBins-1] * Params.dataPath.rangeIdxToMeters;
%
%            posX = range' * sind(theta');
%            posY = range' * cosd(theta');
%            QQ=fftshift(abs(Q),1);
%            QQ=QQ.';
%            QQ=QQ(:,2:end);
%            xlin=linspace(-range_width,range_width,200);
%            ylin=linspace(0,range_depth,200);
%            [X,Y]=meshgrid(xlin,ylin);
%            warning off
%            Z=griddata(posX,posY,fliplr(QQ),X,Y,'linear');
%            warning on
%            surf(Params.guiMonitor.rangeAzimuthHeatMapFigHnd, xlin,ylin,Z);
%            shading INTERP
%            view([0,90])
%            set(Params.guiMonitor.rangeAzimuthHeatMapFigHnd,'Color',[0 0 0.5]);
%%             axis('equal')
%%             axis([-range_width range_width 0 range_depth])
%%             xlabel('Distance along lateral axis (meters)');
%%             ylabel('Distance along longitudinal axis (meters)');
%%             title('Azimuth-Range Heatmap')
        end


        guiProcTime = round(toc(tStart) * 1e3);
        if debugFlag
            fprintf('processing time %f secs \n',toc(tStart));
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

    pause(0.001);


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

function [] = dispError()
disp('error!');
return

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

function [sphandle] = configureCliPort(comportPnum)
    %if ~isempty(instrfind('Type','serial'))
    %    disp('Serial port(s) already open. Re-initializing...');
    %    delete(instrfind('Type','serial'));  % delete open serial ports.
    %end
    comportnum_str=['COM' num2str(comportPnum)]
    sphandle = serial(comportnum_str,'BaudRate',115200);
    set(sphandle,'Parity','none')
    set(sphandle,'Terminator','LF')


    fopen(sphandle);
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


function []=ComputeCR_SNR(noiseEnergy,rp,Params)

cr_range=floor([1.0 5.0]/Params.dataPath.rangeIdxToMeters);

[maxVal  max_idx]=max(rp(cr_range(1): cr_range(2)));

noiseSigma=sqrt(noiseEnergy/(256*2*8));

noiseSigma_dB=10*log10(abs(noiseSigma));

maxValdB=10*maxVal/(log2(10)*2^9);

SNRdB=maxValdB-noiseSigma_dB;
dist= (max_idx+cr_range(1)-1)*Params.dataPath.rangeIdxToMeters;
%[SNRdB dist]
%keyboard
return


% function checkbox1_Callback(hObject, eventdata, handles)
% global ELEV_VIEW
% % hObject    handle to checkbox1 (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
%
% % Hint: get(hObject,'Value') returns toggle state of checkbox1
%
% if (get(hObject,'Value') == get(hObject,'Max'))
%   ELEV_VIEW = 3;
% else
%   ELEV_VIEW = 2;
% end

function checkbox_LinRangeProf_Callback(hObject, eventdata, handles)
global rangeProfileLinearScale
global Params
if (get(hObject,'Value') == get(hObject,'Max'))
  rangeProfileLinearScale = 1;
else
  rangeProfileLinearScale = 0;
end
global maxRngProfYaxis
global maxRangeProfileYaxisLin
global maxRangeProfileYaxisLog
if rangeProfileLinearScale == 1
    maxRngProfYaxis = maxRangeProfileYaxisLin;
else
    maxRngProfYaxis = maxRangeProfileYaxisLog;
end
Params.guiMonitor.logMagRangeUpdate = 1;
return

function checkbox_DopRngHeatMap_Callback(hObject, eventdata, handles)
global dopRngHeatMapFlag

if (get(hObject,'Value') == get(hObject,'Max'))
    dopRngHeatMapFlag = 1;
else
    dopRngHeatMapFlag = 0;
end
return


%Display Chirp parameters in table on screen
function displayChirpParams(Params)
global hndChirpParamTable
global StatsInfo
global guiProcTime
    if hndChirpParamTable ~= 0
        delete(hndChirpParamTable);
        hndChirpParamTable = 0;
    end

    dat =  {'Start Frequency (Ghz)', Params.profileCfg.startFreq;...
            'Slope (MHz/us)', Params.profileCfg.freqSlopeConst;...
            'Samples per chirp', Params.profileCfg.numAdcSamples;...
            'Chirps per frame',  Params.dataPath.numChirpsPerFrame;...
            'Sampling rate (Msps)', Params.profileCfg.digOutSampleRate / 1000;...
            'Bandwidth (GHz)', Params.profileCfg.freqSlopeConst * Params.profileCfg.numAdcSamples /...
                               Params.profileCfg.digOutSampleRate;...
            'Range resolution (m)', Params.dataPath.rangeResolutionMeters;...
            'Velocity resolution (m/s)', Params.dataPath.dopplerResolutionMps;...
            'Number of Tx (MIMO)', Params.dataPath.numTxAnt;...
            'Frame periodicity (msec)', Params.frameCfg.framePeriodicity;...
            'InterFrameProcessingTime (usec)', StatsInfo.interFrameProcessingTime; ...
            'transmitOutputTime (usec)', StatsInfo.transmitOutputTime; ...
            'interFrameProcessingMargin (usec)', StatsInfo.interFrameProcessingMargin; ...
            'InterChirpProcessingMargin (usec)', StatsInfo.interChirpProcessingMargin; ...
            'GuiProcTime (msec)', guiProcTime;
            };

    columnname =   {'___________Parameter (Units)___________', 'Value'};
    columnformat = {'char', 'numeric'};

    t = uitable('Units','normalized','Position',...
                [0.05 0.55 0.8/Params.guiMonitor.numFigCol 0.4], 'Data', dat,...
                'ColumnName', columnname,...
                'ColumnFormat', columnformat,...
                'ColumnWidth', 'auto',...
                'RowName',[]);

    %Center table in lower left quarter
    tPos = get(t, 'Extent');
    if(tPos(4) > 0.9*1/2)
        tPos(4) = 0.9*1/2;
    end
    if(tPos(3) > 0.9*1/Params.guiMonitor.numFigCol)
        tPos(3) = 0.9*1/Params.guiMonitor.numFigCol;
    end
    tPos(1) = 1/Params.guiMonitor.numFigCol / 2 - tPos(3)/2;
    tPos(2) = 1/2 + 1/2 / 2 - tPos(4)/2;
    set(t, 'Position',tPos)

    hndChirpParamTable = t;
return

function UpdateDisplayTable(Params)
global hndChirpParamTable
global StatsInfo
global guiProcTime
global Header
    t = hndChirpParamTable;
    dat = get(t, 'Data');
    row=11;
    dat{row, 2} = StatsInfo.interFrameProcessingTime;
    dat{row+1, 2} = StatsInfo.transmitOutputTime;
    dat{row+2, 2} = StatsInfo.interFrameProcessingMargin;
    dat{row+3, 2} = StatsInfo.interChirpProcessingMargin;
    dat{row+4, 2} = guiProcTime;
    set(t,'Data', dat);
return

function [y] = pow2roundup (x)
    y = 1;
    while x > y
        y = y * 2;
    end
return

function myKeyPressFcn(hObject, event)
    global EXIT_KEY_PRESSED
    if lower(event.Key) == 'q'
        EXIT_KEY_PRESSED  = 1;
    end

return

function Resize_clbk(hObject, event)
global Params
displayChirpParams(Params);
return

function []=plotGrid(R,range_grid_arc)

sect_width=pi/12;
offset_angle=pi/6:sect_width:5*pi/6;
r=[0:range_grid_arc:R];
w=linspace(pi/6,5*pi/6,128);

for n=2:length(r)
    plot(real(r(n)*exp(1j*w)),imag(r(n)*exp(1j*w)),'color',[0.5 0.5 0.5], 'linestyle', ':')
end


for n=1:length(offset_angle)
    plot(real([0 R]*exp(1j*offset_angle(n))),imag([0 R]*exp(1j*offset_angle(n))),'color',[0.5 0.5 0.5], 'linestyle', ':')
end
return

function [Header, idx] = getHeader(bytevec, idx)
    global platformType
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
%     if ~(platformType == hex2dec('a1443'))
%         Header.subframeNumber = sum(bytevec(idx+[1:4]) .* word);
%         idx = idx + 4;
%     end
return

function [tlv, idx] = getTlv(bytevec, idx)
    word = [1 256 65536 16777216]';
    tlv.type = sum(bytevec(idx+(1:4)) .* word);
    idx = idx + 4;
    tlv.length = sum(bytevec(idx+(1:4)) .* word);
    idx = idx + 4;
return

function [detObj, idx] = getDetObj(bytevec, idx, tlvLen, rangeIdxToMeters, dopplerResolutionMps, numDopplerBins)
    global OBJ_STRUCT_SIZE_BYTES;
    detObj =[];
    detObj.numObj = 0;
    if tlvLen > 0
        %Get detected object descriptor
        word = [1 256]';
        detObj.numObj = sum(bytevec(idx+(1:2)) .* word);
        idx = idx + 2;
        xyzQFormat = 2^sum(bytevec(idx+(1:2)) .* word);
        idx = idx + 2;

        %Get detected array of detected objects
        bytes = bytevec(idx+(1:detObj.numObj*OBJ_STRUCT_SIZE_BYTES));
        idx = idx + detObj.numObj*OBJ_STRUCT_SIZE_BYTES;

        bytes = reshape(bytes, OBJ_STRUCT_SIZE_BYTES, detObj.numObj);
        detObj.speedIdx = (bytes(1,:)+bytes(2,:)*256);
        detObj.x = bytes(3,:)+bytes(4,:)*256;
        detObj.y = bytes(5,:)+bytes(6,:)*256;
        detObj.z = bytes(7,:)+bytes(8,:)*256;
        detObj.x( detObj.x > 32767) =  detObj.x( detObj.x > 32767) - 65536;
        detObj.y( detObj.y > 32767) =  detObj.y( detObj.y > 32767) - 65536;
        detObj.z( detObj.z > 32767) =  detObj.z( detObj.z > 32767) - 65536;
        detObj.x =  detObj.x / xyzQFormat;
        detObj.y =  detObj.y / xyzQFormat;
        detObj.z =  detObj.z / xyzQFormat;
    end
return

function [cluster, idx] = getClusters(bytevec, idx, tlvLen)
    global CLS_STRUCT_SIZE_BYTES;
    global MIN_CLUSTER_SIZE;
    cluster = [];

    if tlvLen > 0
        %Get detected object descriptor
        word = [1 256]';
        cluster.numCl = sum(bytevec(idx+(1:2)) .* word);
        idx = idx + 2;
        xyzQFormat = 2^sum(bytevec(idx+(1:2)) .* word);
        idx = idx + 2;

        %Get array of clusters
        bytes = bytevec(idx+(1:cluster.numCl*CLS_STRUCT_SIZE_BYTES));
        idx = idx + cluster.numCl*CLS_STRUCT_SIZE_BYTES;

        bytes = reshape(bytes, CLS_STRUCT_SIZE_BYTES, cluster.numCl);
        cluster.x = bytes(1,:)+bytes(2,:)*256;
        cluster.y = bytes(3,:)+bytes(4,:)*256;
        cluster.z = bytes(5,:)+bytes(6,:)*256;
        cluster.xsize = bytes(7,:)+bytes(8,:)*256;
        cluster.ysize = bytes(9,:)+bytes(10,:)*256;
        cluster.zsize = bytes(11,:)+bytes(12,:)*256;
        cluster.x( cluster.x > 32767) =  cluster.x( cluster.x > 32767) - 65536;
        cluster.y( cluster.y > 32767) =  cluster.y( cluster.y > 32767) - 65536;
        cluster.z( cluster.z > 32767) =  cluster.z( cluster.z > 32767) - 65536;
        cluster.x = cluster.x / xyzQFormat;
        cluster.y = cluster.y / xyzQFormat;
        cluster.z = cluster.z / xyzQFormat;
        cluster.xsize = cluster.xsize / xyzQFormat;
        cluster.ysize = cluster.ysize / xyzQFormat;
        cluster.zsize = cluster.zsize / xyzQFormat;
        cluster.numZero = 0;

        for i=1:cluster.numCl

          if (cluster.xsize(i) == 0) && (cluster.ysize(i) == 0) && (cluster.zsize(i) == 0)
            cluster.numZero = cluster.numZero + 1;
            cluster.zeromap(i) = 1;
          else
            cluster.zeromap(i) = 0;
          end

          % A size of zero means a single point. Set these to a minimum size for plotting
          if (cluster.xsize(i) < MIN_CLUSTER_SIZE)
            cluster.xsize(i) = MIN_CLUSTER_SIZE;
          end
          if (cluster.ysize(i) < MIN_CLUSTER_SIZE)
            cluster.ysize(i) = MIN_CLUSTER_SIZE;
          end
          if (cluster.zsize(i) < MIN_CLUSTER_SIZE)
            cluster.zsize(i) = MIN_CLUSTER_SIZE;
          end
        end
    end
return

function [rp, idx] = getRangeProfile(bytevec, idx, len)
    rp = bytevec(idx+(1:len));
    idx = idx + len;
    rp=rp(1:2:end)+rp(2:2:end)*256;
return

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
function [rangeAzimuth, idx] = getHeatMap(bytevec, idx, numRangeBins, numAngleBins) %==>>
    len = numRangeBins * numAngleBins * 4;
    rangeAzimuth = bytevec(idx+1:idx+len);
    idx = idx + len;

    % group 4 bytes typecase to single
    rangeAzimuth = typecast(uint8(rangeAzimuth), 'single');
return

function [rangeDoppler, idx] = getRangeDopplerHeatMap(bytevec, idx, numDopplerBins, numRangeBins)
    len = numDopplerBins * numRangeBins * 2;
    rangeDoppler = bytevec(idx+(1:len));
    idx = idx + len;
    rangeDoppler = rangeDoppler(1:2:end) + rangeDoppler(2:2:end)*256;
    rangeDoppler = reshape(rangeDoppler, numDopplerBins, numRangeBins);
    rangeDoppler = fftshift(rangeDoppler,1);
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

function [sLin, sLog] = dspFftScalCompDoppler(fftMinSize, fftSize)
    sLin = fftMinSize/fftSize;
    sLog = 20*log10(sLin);
return

function [sLin, sLog] = dspFftScalCompRange(fftMinSize, fftSize)
    smin =  (2.^(ceil(log2(fftMinSize)./log2(4)-1)))  ./ (fftMinSize);
    sLin =  (2.^(ceil(log2(fftSize)./log2(4)-1)))  ./ (fftSize);
    sLin = sLin / smin;
    sLog = 20*log10(sLin);
return


function displayPolarHeatmap(rangeAzimuth_2plot, theta, range)

    figure(1)

    cLim = [0, Inf];
    imagesc_polar2(theta, range, rangeAzimuth_2plot, cLim); hold on

    xlabel('Cross-Range [m]');
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

function displayRectangleHeatmap(rangeAzimuth, theta_degree, range, heattype)

    figure(1)

    cLim = [0, Inf];
    imagesc(theta_degree, range, rangeAzimuth, cLim);

    set(gca,'YDir','normal')
    if (heattype == 1)
      xlabel('Azimuth Angle [degree]');
    else
      xlabel('Elevation [m]');
    end
    ylabel('Range [m]');
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

%
% function [] = updateOccupancyGridAll(hAxisOccupancy, hScatter, indxHist, maxHist)
% hPatch = hAxisOccupancy.UserData.hPatch;
% gridHist = hAxisOccupancy.UserData.gridHist;
% gridElev = hAxisOccupancy.UserData.gridElev;
% gridLim = hAxisOccupancy.UserData.gridLim;
% mode = hAxisOccupancy.UserData.mode;
% %gridPrevFrame = sum(gridHist(:,:,1:maxHist),3);
%
% [r c] = size(hPatch);
%
%
% for indx=1:maxHist
%     % clear grid for current frame
%     gridHist(:,:,indx) = zeros(r,c);
%     gridElev(:,:,indx) = zeros(r,c);
%     % convert det pts to polar
%     if (mode == 1)
%         [ptT, ptR] = cart2pol(hScatter(indx).XData, hScatter(indx).YData);
%         ptT(ptT<0)=ptT(ptT<0)+2*pi();
%     else
%         [ptT, ptR] = cart2pol(hScatter(indx).XData, hScatter(indx).ZData);
%         ptT(ptT<0)=ptT(ptT<0)+2*pi();
%     end
%
%     ptT = rad2deg(ptT);
%     % get points within grid bounds
%     validInd = (ptT<=gridLim(2,2) & ptT>=gridLim(2,1)  & ptR<=gridLim(1,2) & ptR>=gridLim(1,1));
%
%     if(~isempty(validInd))
%         if (mode == 1)
%             elev = hScatter(indx).ZData(validInd);
%         else
%             elev = hScatter(indx).YData(validInd);
%         end
%         % convert polar coord to indices
%         pIndX = ceil((ptT(validInd)-gridLim(2,1))./gridLim(2,3));
%         pIndY = ceil((ptR(validInd)-gridLim(1,1))./gridLim(1,3));
%         % update gridHist
%         for i=1:numel(pIndX)
%             if(pIndX(i) >=1 && pIndX(i) <= r && pIndY(i) >=1 && pIndY(i) <= c)
%                gridHist(pIndX(i),pIndY(i),indx) = gridHist(pIndX(i),pIndY(i),indx) + 1;
%                gridElev(pIndX(i),pIndY(i),indx) = gridElev(pIndX(i),pIndY(i),indx) + elev(i);
%             end
%         end
%
%     end
% end
% %compare frames
% updateGrid = sum(gridHist(:,:,1:maxHist),3);
% meanElev = mean(gridElev(:,:,1:maxHist),3);
% c = parula(10);
% %update patch
% for i=1:r
%     for j=1:c
%         if(updateGrid(i,j))
%            switch hAxisOccupancy.UserData.typePatchFill
%                 case 1
%                     hPatch{i,j}.FaceVertexCData = meanElev(i,j);
%                     disp(meanElev(i,j))
%                     %hPatch{i,j}.FaceVertexAlphaData = 0.2;    % Set constant transparency
%                 case 2
%                     hPatch{i,j}.FaceVertexCData = meanElev(i,j);
%                     disp(meanElev(i,j))
%                     %hPatch{i,j}.FaceVertexAlphaData = min(0.1*updateGrid(i,j),1); % Scale with number of points
%            end
%            hPatch{i,j}.FaceAlpha = 'flat' ;
%         else
%            hPatch{i,j}.FaceColor = 'none';
%         end
%     end
% end

function [] = updateOccupancyGridAll(hAxisOccupancy, hScatter, indxHist, maxHist)
hPatch = hAxisOccupancy.UserData.hPatch;
gridHist = hAxisOccupancy.UserData.gridHist;
gridElev = hAxisOccupancy.UserData.gridElev;
gridLim = hAxisOccupancy.UserData.gridLim;
mode = hAxisOccupancy.UserData.mode;
%gridPrevFrame = sum(gridHist(:,:,1:maxHist),3);

[r c] = size(hPatch);

for indx=1:maxHist
    % clear grid for current frame
    gridHist(:,:,indx) = zeros(r,c);
    gridElev(:,:,indx) = zeros(r,c);
    % convert det pts to polar
    if (mode == 1)
        [ptT, ptR] = cart2pol(hScatter(indx).XData, hScatter(indx).YData);
    else
        [ptT, ptR] = cart2pol(hScatter(indx).XData, hScatter(indx).ZData);
        ptT(ptT<0)=ptT(ptT<0)+2*pi();
    end

    ptT = rad2deg(ptT);
    % get points within grid bounds
    validInd = (ptT<=gridLim(2,2) & ptT>=gridLim(2,1)  & ptR<=gridLim(1,2) & ptR>=gridLim(1,1));

    if(~isempty(validInd))
        if (mode == 1)
            elev = hScatter(indx).CData(validInd);
        else
            elev = hScatter(indx).CData(validInd);
        end
        % convert polar coord to indices
        pIndX = ceil((ptT(validInd)-gridLim(2,1))./gridLim(2,3));
        pIndY = ceil((ptR(validInd)-gridLim(1,1))./gridLim(1,3)); %mode = 2;
        % update gridHist
        for i=1:numel(pIndX)
            if(pIndX(i) >=1 && pIndX(i) <= r && pIndY(i) >=1 && pIndY(i) <= c)
                %gridElev(pIndX(i),pIndY(i),indx) = mean([gridElev(pIndX(i),pIndY(i),indx).*gridHist(pIndX(i),pIndY(i),indx) elev(i)]);
                gridElev(pIndX(i),pIndY(i),indx) = gridElev(pIndX(i),pIndY(i),indx) + elev(i);
                gridHist(pIndX(i),pIndY(i),indx) = gridHist(pIndX(i),pIndY(i),indx) + 1;

            end
        end

    end
end


function [] = drawClusterCubes(hplot, hClust, cluster)

  global MIN_NORMAL_DIST;
  global lastClustCount;

  %  Define the vertices of the unit cube
  ver = [1 1 0;
         0 1 0;
         0 1 1;
         1 1 1;
         0 0 1;
         1 0 1;
         1 0 0;
         0 0 0];

  count = 1;

  for idx=1:cluster.numCl

    X = cluster.x(idx);
    Y = cluster.y(idx);
    Z = cluster.z(idx);
    xsize = cluster.xsize(idx);
    ysize = cluster.ysize(idx);
    zsize = cluster.zsize(idx);

    if (abs(Z+zsize) < hplot.ZLim(2)) && ((Y+ysize) < hplot.YLim(2))
      pos = [X, Y, Z];
      siz = [xsize, ysize, zsize];
      orig= pos - (siz/2);       % Get the origin of cube so that P is at center

      if (X < MIN_NORMAL_DIST) && (Y < MIN_NORMAL_DIST)
        clr = [1 0 0];
      else
        clr = [0 1 0];
      end

      cube = [ver(:,1)*xsize+orig(1),ver(:,2)*ysize+orig(2),ver(:,3)*zsize+orig(3)];
%      hold on
      set(hClust(count), 'Vertices', cube, 'EdgeColor', clr);
      count = count + 1;
    end
  end

  if (count <= lastClustCount)
    for idx=count:lastClustCount
      cube = zeros(8, 3);
%      hold on
      set(hClust(idx), 'Vertices', cube, 'EdgeColor', [0 0 0]);
    end
  end

  lastClustCount = count;

return;
