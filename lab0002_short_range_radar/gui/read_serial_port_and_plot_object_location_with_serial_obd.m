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
function [] = read_serial_port_and_plot_object_location_with_serial_obd(comportSnum, dim, record_options, obd_serial)
global  load_parking_assist load_point_cloud_srr load_point_cloud_usrr load_clusters load_trackers view_range use_perspective_projection;
global  platformType MAX_NUM_OBJECTS OBJ_STRUCT_SIZE_BYTES MAX_NUM_CLUSTERS;
global  CLUSTER_STRUCT_SIZE_BYTES TOTAL_PAYLOAD_SIZE_BYTES MAX_NUM_TRACKERS;
global TRACKER_STRUCT_SIZE_BYTES STATS_SIZE_BYTES bytevec_log readUartFcnCntr;
global ELEV_VIEW EXIT_KEY_PRESSED BYTE_VEC_ACC_MAX_SIZE bytevecAcc;
global BYTES_AVAILABLE_FCN_CNT BYTES_AVAILABLE_FLAG bytevecAccLen StatsInfo;
global activeFrameCPULoad interFrameCPULoad guiCPULoad guiProcTime;
global loggingEnable fidLog recordingHandle;

% The SRR demo only works on the 1642.
platformType = hex2dec('a1642');

MMWDEMO_UART_MSG_DETECTED_POINTS    = 1;
MMWDEMO_UART_MSG_CLUSTERS           = 2;
MMWDEMO_UART_MSG_TRACKED_OBJ        = 3;
MMWDEMO_UART_MSG_PARKING_ASSIST     = 4;
MMWDEMO_UART_MSG_STATS = 6;

load_point_cloud_srr = 1;
load_point_cloud_usrr = 1;
load_clusters  = 1;
load_trackers = 1;
view_range = 0;
load_parking_assist = 0;
view_range_curr = view_range;

%% Initialize some constants.
MAX_NUM_OBJECTS = 200;
OBJ_STRUCT_SIZE_BYTES = 8;
MAX_NUM_CLUSTERS = 24;
CLUSTER_STRUCT_SIZE_BYTES = 8;
MAX_NUM_TRACKERS = 24;
TRACKER_STRUCT_SIZE_BYTES = 12;
STATS_SIZE_BYTES = 16;
bytevec_log = zeros(0,1,'single');
readUartFcnCntr = 0;
ELEV_VIEW = 3;
EXIT_KEY_PRESSED = 0;
BYTES_AVAILABLE_FLAG = 0;
BYTES_AVAILABLE_FCN_CNT = 32*8;
BYTE_VEC_ACC_MAX_SIZE  = 2^16;
bytevecAcc = zeros(BYTE_VEC_ACC_MAX_SIZE,1);
bytevecAccLen = 0;

%% Some more Initialisations
StatsInfo.interFrameProcessingTime = 0;
StatsInfo.transmitOutputTime = 0;
StatsInfo.interFrameProcessingMargin = 0;
StatsInfo.interChirpProcessingMargin = 0;
StatsInfo.interFrameCPULoad = 0;
StatsInfo.activeFrameCPULoad = 0;

activeFrameCPULoad = zeros(100,1,'single');
interFrameCPULoad = zeros(100,1,'single');
guiCPULoad = zeros(100,1,'single');
view_range = 0;
guiProcTime = 0;

displayUpdateCntr   = 0;
timeout_ctr         = 0;
bytevec_cp_max_len  = 2^15;
bytevec_cp          = zeros(bytevec_cp_max_len,1,'uint8');
bytevec_cp_len      = 0;
packetNumberPrev    = 0;
loggingEnable       = 0;
fidLog              = 0;
use_perspective_projection = 0;
prev_use_perspective_projection = use_perspective_projection;

%% Parse CLI parameters
Params = generate_params_for_SRR();
%% Configure monitoring UART port
sphandle = configureSport(comportSnum);

obdfhandle = fopen(obd_serial.file, 'r');

recordingHandle.enable = record_options.record;
if recordingHandle.enable == 1
    recordingHandle.size = 2^28;
    recordingHandle.indx = 1;
    recordingHandle.array = zeros(recordingHandle.size,1,'uint8');
end

guiMonitor = gui_initializer(Params, dim);

guiMonitor.detectedObjectsACumulativeData.Strong.x = [];
guiMonitor.detectedObjectsACumulativeData.Mid.x = [];
guiMonitor.detectedObjectsACumulativeData.Weak.x = [];

guiMonitor.detectedObjectsACumulativeData.Strong.y = [];
guiMonitor.detectedObjectsACumulativeData.Mid.y = [];
guiMonitor.detectedObjectsACumulativeData.Weak.y = [];

guiMonitor.detectedObjectsBCumulativeData.Strong.x = [];
guiMonitor.detectedObjectsBCumulativeData.Mid.x = [];
guiMonitor.detectedObjectsBCumulativeData.Weak.x = [];

guiMonitor.detectedObjectsBCumulativeData.Strong.y = [];
guiMonitor.detectedObjectsBCumulativeData.Mid.y = [];
guiMonitor.detectedObjectsBCumulativeData.Weak.y = [];



guiMonitor.trackedObjCumulativeData.x = [];
guiMonitor.trackedObjCumulativeData.y = [];
guiMonitor.clustersCumulativeDataA.x = [];
guiMonitor.clustersCumulativeDataA.y = [];
guiMonitor.clustersCumulativeDataB.x = [];
guiMonitor.clustersCumulativeDataB.y = [];

currOffset.x = 0;
currOffset.y = 0;
frameInit = -1;
barker_code = char([2 1 4 3 6 5 8 7]);
magicNotOkCntr = 0;


%-------------------- Main Loop ------------------------
while ((~EXIT_KEY_PRESSED) )
    %Read bytes
    readUartCallbackFcn(sphandle, 0);
    if ((currOffset.x > dim.max_dist_x) || (currOffset.y > dim.max_dist_y))
        guiMonitor.detectedObjectsACumulativeData.Strong.x = [];
        guiMonitor.detectedObjectsACumulativeData.Mid.x = [];
        guiMonitor.detectedObjectsACumulativeData.Weak.x = [];

        guiMonitor.detectedObjectsACumulativeData.Strong.y = [];
        guiMonitor.detectedObjectsACumulativeData.Mid.y = [];
        guiMonitor.detectedObjectsACumulativeData.Weak.y = [];

        guiMonitor.detectedObjectsBCumulativeData.Strong.x = [];
        guiMonitor.detectedObjectsBCumulativeData.Mid.x = [];
        guiMonitor.detectedObjectsBCumulativeData.Weak.x = [];

        guiMonitor.detectedObjectsBCumulativeData.Strong.y = [];
        guiMonitor.detectedObjectsBCumulativeData.Mid.y = [];
        guiMonitor.detectedObjectsBCumulativeData.Weak.y = [];

        guiMonitor.trackedObjCumulativeData.x = [];
        guiMonitor.trackedObjCumulativeData.y = [];
        guiMonitor.clustersCumulativeDataA.x = [];
        guiMonitor.clustersCumulativeDataA.y = [];
        guiMonitor.clustersCumulativeDataB.x = [];
        guiMonitor.clustersCumulativeDataB.y = [];
        
        currOffset.x = 0;
        currOffset.y = 0;

    end
    
    if BYTES_AVAILABLE_FLAG == 1
        BYTES_AVAILABLE_FLAG = 0;
        %fprintf('bytevec_cp_len, bytevecAccLen = %d %d \n',bytevec_cp_len, bytevecAccLen)
        if (bytevec_cp_len + bytevecAccLen) < bytevec_cp_max_len
            bytevec_cp(bytevec_cp_len+1:bytevec_cp_len + bytevecAccLen) = bytevecAcc(1:bytevecAccLen);
            bytevec_cp_len = bytevec_cp_len + bytevecAccLen;
            bytevecAccLen = 0;
        else
            fprintf('Error: Buffer overflow, bytevec_cp_len, bytevecAccLen = %d %d \n',bytevec_cp_len, bytevecAccLen)
            bytevecAccLen = 0;
            bytevec_cp_len = 0;
        end
    end
    
    bytevecStr = (bytevec_cp);
    magicOk = 0;
	% if the bytevecStr is atleast as large as the header, check if it contains the header. 
    if (bytevec_cp_len > 72) && (size(bytevecStr,2) == 1)
        startIdx = strfind(bytevecStr', barker_code);
    else
        startIdx = [];
    end
    if ~isempty(startIdx)
        if startIdx(1) > 1
            bytevec_cp(1: bytevec_cp_len-(startIdx(1)-1)) = bytevec_cp(startIdx(1):bytevec_cp_len);
            bytevec_cp_len = bytevec_cp_len - (startIdx(1)-1);
        end
        if bytevec_cp_len < 0
            fprintf('Error: %d %d \n',bytevec_cp_len, bytevecAccLen)
            bytevec_cp_len = 0;
        end
        
        packetlenNum = single(bytevec_cp(8+4+[1:4]));
        totalPacketLen = sum(packetlenNum .* [1 256 65536 16777216]');
        if bytevec_cp_len >= totalPacketLen
            magicOk = 1;
        else
            magicOk = 0;
        end
    end
    
    byteVecIdx = 0;
    if(magicOk == 1)
        tStart = tic;
        bytevec_cp_flt = single(bytevec_cp);
        [Header, byteVecIdx] = getHeader(bytevec_cp_flt, byteVecIdx);
        sfIdx = Header.subframeNumber+1;
        if (sfIdx > 2) || (Header.numDetectedObj > MAX_NUM_OBJECTS)
            continue;
        end
        detObj.numObj = 0;
        clusterObj.numObj = 0;
        trackedObj.numObj = 0;
        if frameInit < 0
            frameInit = Header.frameNumber;
        end
        
        timePeriod = (Header.frameNumber - frameInit)*Params(1).frameCfg.framePeriodicity/1000 + obd_serial.offset;
        
        fseek(obdfhandle, -1, 1);
        carVel= fread(obdfhandle,1,'uint8=>single')/3.6;
        
        dist_travelled = carVel*Params(1).frameCfg.framePeriodicity/1000;
        currOffset.x = currOffset.x + sind(obd_serial.orientation)*dist_travelled;
        currOffset.y = currOffset.y + cosd(obd_serial.orientation)*dist_travelled;
        
        for tlvIdx = 1:Header.numTLVs
            [tlv, byteVecIdx] = getTlv(bytevec_cp_flt, byteVecIdx);
            switch tlv.type
                case MMWDEMO_UART_MSG_DETECTED_POINTS
                    if tlv.length >= OBJ_STRUCT_SIZE_BYTES
                        [detObj, byteVecIdx] = getDetObj(bytevec_cp_flt, ...
                            byteVecIdx, ...
                            tlv.length);
                    end
                case MMWDEMO_UART_MSG_CLUSTERS
                    if tlv.length >= CLUSTER_STRUCT_SIZE_BYTES
                        [clusterObj, byteVecIdx] = getClusters(bytevec_cp_flt, ...
                            byteVecIdx, ...
                            tlv.length);
                    end
                case MMWDEMO_UART_MSG_TRACKED_OBJ
                    if tlv.length >= TRACKER_STRUCT_SIZE_BYTES
                        [trackedObj, byteVecIdx] = getTrackers(bytevec_cp_flt, byteVecIdx, tlv.length);
                    end
                    
                case MMWDEMO_UART_MSG_PARKING_ASSIST
                    [parkingAssistRangeBins, byteVecIdx] = getParkingAssistBins(bytevec_cp_flt, byteVecIdx, tlv.length);
                    
                case MMWDEMO_UART_MSG_STATS
                    [StatsInfo, byteVecIdx] = getStatsInfo(bytevec_cp_flt, byteVecIdx);
                    %fprintf('StatsInfo: %d, %d, %d %d \n', StatsInfo.interFrameProcessingTime, StatsInfo.transmitOutputTime, StatsInfo.interFrameProcessingMargin, StatsInfo.interChirpProcessingMargin);
                    displayUpdateCntr = displayUpdateCntr + 1;
                    interFrameCPULoad = [interFrameCPULoad(2:end); StatsInfo.interFrameCPULoad];
                    activeFrameCPULoad = [activeFrameCPULoad(2:end); StatsInfo.activeFrameCPULoad];
                    guiCPULoad = [guiCPULoad(2:end); 100*guiProcTime/Params(1).frameCfg.framePeriodicity];
                    if displayUpdateCntr == 40
                        UpdateDisplayTable(Params);
                        displayUpdateCntr = 0;
                    end
                otherwise
            end
        end
        
        byteVecIdx = Header.totalPacketLen;
        
        if ((Header.frameNumber - packetNumberPrev) ~= 1) && (packetNumberPrev ~= 0)
            fprintf('Error: Packets lost: %d, current frame num = %d \n', (Header.frameNumber - packetNumberPrev - 1), Header.frameNumber)
        end
        packetNumberPrev = Header.frameNumber;
        
        
        % 1. Detected objects
        if sfIdx == 1
            if (detObj.numObj > 0) && load_point_cloud_srr
                detObj.Weak.x = detObj.Weak.x + currOffset.x;
                detObj.Weak.y = detObj.Weak.y + currOffset.y;
                guiMonitor.detectedObjectsACumulativeData.Weak.x = [guiMonitor.detectedObjectsACumulativeData.Weak.x detObj.Weak.x];
                guiMonitor.detectedObjectsACumulativeData.Weak.y = [guiMonitor.detectedObjectsACumulativeData.Weak.y detObj.Weak.y];
                
                set(guiMonitor.detectedObjectsPlotHndA.Weak, 'Xdata', guiMonitor.detectedObjectsACumulativeData.Weak.x, 'Ydata', guiMonitor.detectedObjectsACumulativeData.Weak.y);
                set(guiMonitor.detectedObjectsRngDopPlotHndA.Weak, 'Xdata', detObj.Weak.range, 'Ydata', detObj.Weak.doppler);
                
                if numel(detObj.Mid.x)
                    detObj.Mid.x = detObj.Mid.x + currOffset.x;
                    detObj.Mid.y = detObj.Mid.y + currOffset.y;
                    
                    guiMonitor.detectedObjectsACumulativeData.Mid.x = [guiMonitor.detectedObjectsACumulativeData.Mid.x detObj.Mid.x];
                    guiMonitor.detectedObjectsACumulativeData.Mid.y = [guiMonitor.detectedObjectsACumulativeData.Mid.y detObj.Mid.y];
                    
                    set(guiMonitor.detectedObjectsPlotHndA.Mid, 'Xdata', guiMonitor.detectedObjectsACumulativeData.Mid.x, 'Ydata', guiMonitor.detectedObjectsACumulativeData.Mid.y);
                    set(guiMonitor.detectedObjectsRngDopPlotHndA.Mid, 'Xdata', detObj.Mid.range, 'Ydata', detObj.Mid.doppler);
                end
                
                if numel(detObj.Strong.x)
                    
                    detObj.Strong.x = detObj.Strong.x + currOffset.x;
                    detObj.Strong.y = detObj.Strong.y + currOffset.y;
                    
                    guiMonitor.detectedObjectsACumulativeData.Strong.x = [guiMonitor.detectedObjectsACumulativeData.Strong.x detObj.Strong.x];
                    guiMonitor.detectedObjectsACumulativeData.Strong.y = [guiMonitor.detectedObjectsACumulativeData.Strong.y detObj.Strong.y];
                    
                    set(guiMonitor.detectedObjectsPlotHndA.Strong, 'Xdata', guiMonitor.detectedObjectsACumulativeData.Strong.x, 'Ydata', guiMonitor.detectedObjectsACumulativeData.Strong.y);
                    set(guiMonitor.detectedObjectsRngDopPlotHndA.Strong, 'Xdata', detObj.Strong.range, 'Ydata', detObj.Strong.doppler);
                end
            else
                set(guiMonitor.detectedObjectsPlotHndA.Weak, 'Xdata', inf, 'Ydata', inf);
                set(guiMonitor.detectedObjectsRngDopPlotHndA.Weak, 'Xdata', inf, 'Ydata', inf);
                
                set(guiMonitor.detectedObjectsPlotHndA.Mid, 'Xdata', inf, 'Ydata', inf);
                set(guiMonitor.detectedObjectsRngDopPlotHndA.Mid, 'Xdata', inf, 'Ydata', inf);
                
                set(guiMonitor.detectedObjectsPlotHndA.Strong, 'Xdata', inf, 'Ydata', inf);
                set(guiMonitor.detectedObjectsRngDopPlotHndA.Strong, 'Xdata', inf, 'Ydata', inf);
            end
        else
            if (detObj.numObj > 0) && load_point_cloud_usrr
                detObj.Weak.x = detObj.Weak.x + currOffset.x;
                detObj.Weak.y = detObj.Weak.y + currOffset.y;
                guiMonitor.detectedObjectsBCumulativeData.Weak.x = [guiMonitor.detectedObjectsBCumulativeData.Weak.x detObj.Weak.x];
                guiMonitor.detectedObjectsBCumulativeData.Weak.y = [guiMonitor.detectedObjectsBCumulativeData.Weak.y detObj.Weak.y];
                
                set(guiMonitor.detectedObjectsPlotHndB.Weak, 'Xdata', guiMonitor.detectedObjectsBCumulativeData.Weak.x, 'Ydata', guiMonitor.detectedObjectsBCumulativeData.Weak.y);
                set(guiMonitor.detectedObjectsRngDopPlotHndB.Weak, 'Xdata', detObj.Weak.range, 'Ydata', detObj.Weak.doppler);
                
                if numel(detObj.Mid.x)
                    detObj.Mid.x = detObj.Mid.x + currOffset.x;
                    detObj.Mid.y = detObj.Mid.y + currOffset.y;
                    
                    guiMonitor.detectedObjectsBCumulativeData.Mid.x = [guiMonitor.detectedObjectsBCumulativeData.Mid.x detObj.Mid.x];
                    guiMonitor.detectedObjectsBCumulativeData.Mid.y = [guiMonitor.detectedObjectsBCumulativeData.Mid.y detObj.Mid.y];
                    
                    set(guiMonitor.detectedObjectsPlotHndB.Mid, 'Xdata', guiMonitor.detectedObjectsBCumulativeData.Mid.x, 'Ydata', guiMonitor.detectedObjectsBCumulativeData.Mid.y);
                    set(guiMonitor.detectedObjectsRngDopPlotHndB.Mid, 'Xdata', detObj.Mid.range, 'Ydata', detObj.Mid.doppler);
                end
                
                if numel(detObj.Strong.x)
                    
                    detObj.Strong.x = detObj.Strong.x + currOffset.x;
                    detObj.Strong.y = detObj.Strong.y + currOffset.y;
                    
                    guiMonitor.detectedObjectsBCumulativeData.Strong.x = [guiMonitor.detectedObjectsBCumulativeData.Strong.x detObj.Strong.x];
                    guiMonitor.detectedObjectsBCumulativeData.Strong.y = [guiMonitor.detectedObjectsBCumulativeData.Strong.y detObj.Strong.y];
                    
                    set(guiMonitor.detectedObjectsPlotHndB.Strong, 'Xdata', guiMonitor.detectedObjectsBCumulativeData.Strong.x, 'Ydata', guiMonitor.detectedObjectsBCumulativeData.Strong.y);
                    set(guiMonitor.detectedObjectsRngDopPlotHndB.Strong, 'Xdata', detObj.Strong.range, 'Ydata', detObj.Strong.doppler);
                end
            else
                set(guiMonitor.detectedObjectsPlotHndB.Weak, 'Xdata', inf, 'Ydata', inf);
                set(guiMonitor.detectedObjectsRngDopPlotHndB.Weak, 'Xdata', inf, 'Ydata', inf);
                
                set(guiMonitor.detectedObjectsPlotHndB.Mid, 'Xdata', inf, 'Ydata', inf);
                set(guiMonitor.detectedObjectsRngDopPlotHndB.Mid, 'Xdata', inf, 'Ydata', inf);
                
                set(guiMonitor.detectedObjectsPlotHndB.Strong, 'Xdata', inf, 'Ydata', inf);
                set(guiMonitor.detectedObjectsRngDopPlotHndB.Strong, 'Xdata', inf, 'Ydata', inf);
            end
        end
        
        % 2. Clusters.
        if sfIdx == 2
            if (clusterObj.numObj > 0) && load_clusters
                clusterObj.y_loc = clusterObj.y_loc + currOffset.y;
                clusterObj.x_loc = clusterObj.x_loc + currOffset.x;
                guiMonitor.clustersCumulativeDataB.x = [guiMonitor.clustersCumulativeDataB.x; clusterObj.x_loc];
                guiMonitor.clustersCumulativeDataB.y = [guiMonitor.clustersCumulativeDataB.y; clusterObj.y_loc];
                
                set(guiMonitor.clustersPlotHndB, 'Xdata', guiMonitor.clustersCumulativeDataB.x, 'Ydata', guiMonitor.clustersCumulativeDataB.y);
            else
                set(guiMonitor.clustersPlotHndB, 'Xdata', inf, 'Ydata', inf);
            end
        end
        
        
        % 3. Tracking
        if (trackedObj.numObj > 0) && load_trackers
            trackedObj.y = trackedObj.y + currOffset.y;
            trackedObj.x = trackedObj.x + currOffset.x;
            
            guiMonitor.trackedObjCumulativeData.x = [guiMonitor.trackedObjCumulativeData.x  trackedObj.x];
            guiMonitor.trackedObjCumulativeData.y = [guiMonitor.trackedObjCumulativeData.y  trackedObj.y];
            
            set(guiMonitor.trackedObjPlotHnd, 'Xdata', guiMonitor.trackedObjCumulativeData.x, 'Ydata', guiMonitor.trackedObjCumulativeData.y);
            set(guiMonitor.trackedObjRngDop, 'Xdata', trackedObj.range, 'Ydata', trackedObj.doppler);
        else
            if sfIdx == 1
                set(guiMonitor.trackedObjPlotHnd, 'Xdata', inf, 'Ydata', inf);
                set(guiMonitor.trackedObjRngDop, 'Xdata', inf, 'Ydata', inf);
            end
        end
        
        % 4. Parking Assist
        if sfIdx == 2
            if load_parking_assist
                parkingAssistRangeBins.x = parkingAssistRangeBins.x + currOffset.x;
                parkingAssistRangeBins.y = parkingAssistRangeBins.y + currOffset.y;
                set(guiMonitor.parkingAssistRangeBinsHnd, 'Xdata', parkingAssistRangeBins.x,'Ydata', parkingAssistRangeBins.y);
            else
                set(guiMonitor.parkingAssistRangeBinsHnd, 'Xdata', inf,'Ydata', inf);
            end
        end
        
        guiProcTime = round(toc(tStart) * 1e3);
    else
        magicNotOkCntr = magicNotOkCntr + 1;
    end
    
    %Remove processed data
    if byteVecIdx > 0
        shiftSize = byteVecIdx;
        bytevec_cp = bytevec_cp(shiftSize+1:bytevec_cp_len);
        bytevec_cp_len = bytevec_cp_len - shiftSize;
        if bytevec_cp_len < 0
            fprintf('Error: bytevec_cp_len < bytevecAccLen, %d %d \n', bytevec_cp_len, bytevecAccLen)
            bytevec_cp_len = 0;
        end
    end
    if bytevec_cp_len > (bytevec_cp_max_len * 7/8)
        bytevec_cp_len = 0;
    end
    
    
    if (view_range_curr ~= view_range)
        if view_range == 0
            range_depth_tmp = dim.max_dist_y;
            range_width_tmp = dim.max_dist_x;
            dopplerRange_tmp = dim.max_vel;
        else
            range_depth_tmp = dim.max_dist_y/5;
            range_width_tmp = dim.max_dist_x/5;
            dopplerRange_tmp = dim.max_vel/5;
        end
        view_range_curr = view_range;
        
        subplot(guiMonitor.detectedObjectsFigHnd);
        axis([-range_width_tmp range_width_tmp 0 range_depth_tmp]);
        subplot(guiMonitor.detectedObjectsRngDopFigHnd);
        axis([0 range_depth_tmp -dopplerRange_tmp dopplerRange_tmp]);
    end
    
    if use_perspective_projection ~= prev_use_perspective_projection
        camproj(guiMonitor.detectedObjectsFigHnd, 'perspective');
        campos(guiMonitor.detectedObjectsFigHnd, [0,0,dim.max_dist_y/2]);
        camtarget([0,dim.max_dist_y*0.33,0]);
        prev_use_perspective_projection = use_perspective_projection;
    end
    tIdleStart = tic;
    
    
    if(toc(tIdleStart) > 2*Params(1).frameCfg.framePeriodicity/1000)
        timeout_ctr=timeout_ctr+1;
        tIdleStart = tic;
    end
    pause(0.005);
    
end
%close and delete handles before exiting
close(guiMonitor.figHnd); % close figure
fclose(sphandle); %close com port (or file)
delete(guiMonitor.figHnd);
if record_options.record == 1
    fid = fopen(record_options.filename_rec,'w');
    fwrite(fid,recordingHandle.array(1:recordingHandle.indx),'uint8');
    fclose(fid);
end


return

function [] = readUartCallbackFcn(obj, event)
global bytevecAcc;
global bytevecAccLen;
global readUartFcnCntr;
global BYTES_AVAILABLE_FLAG
global BYTE_VEC_ACC_MAX_SIZE
global EXIT_KEY_PRESSED
global recordingHandle;

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

if recordingHandle.enable
    recordingHandle.array(recordingHandle.indx+1:recordingHandle.indx+byteCount) = bytevec;
    recordingHandle.indx = recordingHandle.indx + byteCount;
    if (recordingHandle.indx  + 16384 > recordingHandle.size)
        % Exit as soon as the recording buffer is close to being full.
        EXIT_KEY_PRESSED = 1;
    end
end
readUartFcnCntr = readUartFcnCntr + 1;
BYTES_AVAILABLE_FLAG = 1;
return

function [] = dispError()
disp('error!');
return

function [y] = pow2roundup (x)
y = 1;
while x > y
    y = y * 2;
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

function [detObj, idx] = getDetObj(bytevec, idx, tlvLen)
global OBJ_STRUCT_SIZE_BYTES;
detObj =[];
detObj.numObj = 0;
len_bytevec = length(bytevec);
if len_bytevec < idx +4; 
    idx = len_bytevec;
    return;
end
if tlvLen > 0
    %Get detected object descriptor
    word = [1 256]';
    detObj.numObj = sum(bytevec(idx+(1:2)) .* word);
    idx = idx + 2;
    xyzQFormat = 2^sum(bytevec(idx+(1:2)) .* word);
    idx = idx + 2;
    invXyzQFormat = 1/xyzQFormat;
    %Check if the array can be fulfilled. 
    if len_bytevec < idx + detObj.numObj*OBJ_STRUCT_SIZE_BYTES
        detObj.numObj = 0; 
        idx = len_bytevec;
        return;
    end
    bytes = bytevec(idx+(1:detObj.numObj*OBJ_STRUCT_SIZE_BYTES));
    idx = idx + detObj.numObj*OBJ_STRUCT_SIZE_BYTES;
    bytes = reshape(bytes, OBJ_STRUCT_SIZE_BYTES, detObj.numObj);
    detObj.doppler = (bytes(1,:)+bytes(2,:)*256);
    detObj.peakVal = bytes(3,:)+bytes(4,:)*256; %peak value (16-bit=> so constructed from 2 bytes)
    detObj.x = bytes(5,:)+bytes(6,:)*256;
    detObj.y = bytes(7,:)+bytes(8,:)*256;
    detObj.x( detObj.x > 32767) =  detObj.x( detObj.x > 32767) - 65536;
    detObj.y( detObj.y > 32767) =  detObj.y( detObj.y > 32767) - 65536;
    detObj.doppler(detObj.doppler > 32767) = detObj.doppler(detObj.doppler > 32767) -65536;
    detObj.x = detObj.x*invXyzQFormat;
    detObj.y = detObj.y*invXyzQFormat;
    degree = atan2d(detObj.x, detObj.y);
    badDegreeindices  = (abs(degree) > 5);
    
    detObj.y(badDegreeindices) = [];
    detObj.x(badDegreeindices) = [];
    detObj.doppler(badDegreeindices) = [];
    detObj.peakVal(badDegreeindices) = [];
    
    badDetIndx = ( (detObj.y < 3)  & (detObj.peakVal < 3.3e4/8));
    
    detObj.y(badDetIndx) = [];
    detObj.x(badDetIndx) = [];
    detObj.doppler(badDetIndx) = [];
    detObj.peakVal(badDetIndx) = [];
    
    detObj.doppler = detObj.doppler*invXyzQFormat;
    detObj.range = sqrt(detObj.y.*detObj.y + detObj.x.*detObj.x);
    
    
    [~, idxAct] = sort(detObj.peakVal,'descend');
    
    high = 38500/8;
    mid = 34500/8;
    
    idxStrong = idxAct(detObj.peakVal > high);
    detObj.Strong.peakVal = detObj.peakVal(idxStrong);
    detObj.Strong.x = detObj.x(idxStrong);
    detObj.Strong.y = detObj.y(idxStrong);
    detObj.Strong.doppler = detObj.doppler (idxStrong);
    detObj.Strong.range = detObj.range (idxStrong);
    
    idxMid = idxAct((detObj.peakVal < high) & (detObj.peakVal > mid));
    detObj.Mid.peakVal = detObj.peakVal(idxMid);
    detObj.Mid.x = detObj.x(idxMid);
    detObj.Mid.y = detObj.y(idxMid);
    detObj.Mid.doppler = detObj.doppler (idxMid);
    detObj.Mid.range = detObj.range (idxMid);
    
    idxWeak = idxAct((detObj.peakVal < mid));
    detObj.Weak.peakVal = detObj.peakVal(idxWeak);
    detObj.Weak.x = detObj.x(idxWeak);
    detObj.Weak.y = detObj.y(idxWeak);
    detObj.Weak.doppler = detObj.doppler (idxWeak);
    detObj.Weak.range = detObj.range (idxWeak);
end
return

function [clusterObj, idx] = getClusters(bytevec, idx, tlvLen)
global CLUSTER_STRUCT_SIZE_BYTES;
clusterObj =[];
clusterObj.numObj = 0;

len_bytevec = length(bytevec);
if len_bytevec < idx +4; 
    idx = len_bytevec;
    return;
end
if tlvLen > 0
    %Get detected object descriptor
    word = [1 256]';
    clusterObj.numObj = sum(bytevec(idx+(1:2)) .* word);
    idx = idx + 2;
    onebyXyzQFormat = 1/2^sum(bytevec(idx+(1:2)) .* word);
    idx = idx + 2;
    
    %Get detected array of detected objects
    if len_bytevec < idx + (1:clusterObj.numObj*CLUSTER_STRUCT_SIZE_BYTES); 
        clusterObj.numObj = 0;
        idx = len_bytevec;
        return;
    end
    bytes = bytevec(idx+(1:clusterObj.numObj*CLUSTER_STRUCT_SIZE_BYTES));
    idx = idx + clusterObj.numObj*CLUSTER_STRUCT_SIZE_BYTES;
    
    bytes = reshape(bytes, CLUSTER_STRUCT_SIZE_BYTES, clusterObj.numObj);
    x = bytes(1,:)+bytes(2,:)*256;
    y = bytes(3,:)+bytes(4,:)*256;
    x( x > 32767) =  x( x > 32767) - 65536;
    y( y > 32767) =  y( y > 32767) - 65536;
    x = x.*onebyXyzQFormat;
    y = y.*onebyXyzQFormat;
    
    x_size = bytes(5,:)+bytes(6,:)*256;
    y_size = bytes(7,:)+bytes(8,:)*256;
    x_size = x_size.*onebyXyzQFormat;
    y_size = y_size.*onebyXyzQFormat;
    
    x_loc = (repmat(x',[1,6])) + x_size'*([-1 1 1 -1 -1 inf]);
    y_loc = (repmat(y',[1,6])) + y_size'*([-1 -1 1 1 -1 inf]);
    
    x_loc = x_loc';
    y_loc = y_loc';
    
    clusterObj.x_loc = x_loc(:);
    clusterObj.y_loc = y_loc(:);
    
end
return

function [detObj, idx] = getTrackers(bytevec, idx, tlvLen)
global TRACKER_STRUCT_SIZE_BYTES ;
detObj =[];
detObj.numObj = 0;
len_bytevec = length(bytevec);
if len_bytevec < idx +4; 
    idx = len_bytevec;
    return;
end

if tlvLen > 0
    %Get detected object descriptor
    word = [1 256]';
    detObj.numObj = sum(bytevec(idx+(1:2)) .* word);
    idx = idx + 2;
    xyzQFormat = 2^sum(bytevec(idx+(1:2)) .* word);
    idx = idx + 2;
    onebyXyzQFormat = 1/xyzQFormat;
    %Get detected array of detected objects
    if len_bytevec < idx + (1:detObj.numObj*TRACKER_STRUCT_SIZE_BYTES); 
        detObj.numObj = 0;
        idx = len_bytevec;
        return;
    end
    bytes = bytevec(idx+(1:detObj.numObj*TRACKER_STRUCT_SIZE_BYTES ));
    idx = idx + detObj.numObj*TRACKER_STRUCT_SIZE_BYTES ;
    
    bytes = reshape(bytes, TRACKER_STRUCT_SIZE_BYTES , detObj.numObj);
    detObj.x = bytes(1,:)+bytes(2,:)*256;
    detObj.y = bytes(3,:)+bytes(4,:)*256;
    detObj.x( detObj.x > 32767) =  detObj.x( detObj.x > 32767) - 65536;
    detObj.y( detObj.y > 32767) =  detObj.y( detObj.y > 32767) - 65536;
    detObj.x = detObj.x*onebyXyzQFormat;
    detObj.y = detObj.y*onebyXyzQFormat;
    
    detObj.vx = bytes(5,:)+bytes(6,:)*256;
    detObj.vy = bytes(7,:)+bytes(8,:)*256;
    detObj.vx( detObj.vx > 32767) =  detObj.vx( detObj.vx > 32767) - 65536;
    detObj.vy( detObj.vy > 32767) =  detObj.vy( detObj.vy > 32767) - 65536;
    detObj.vx = detObj.vx*onebyXyzQFormat;
    detObj.vy = detObj.vy*onebyXyzQFormat;
    x_size = bytes(9,:)+bytes(10,:)*256;
    y_size = bytes(11,:)+bytes(12,:)*256;
    
    x_size = x_size.*onebyXyzQFormat;
    y_size = y_size.*onebyXyzQFormat;
    
    x_loc = (repmat(detObj.x',[1,6])) + x_size'*([-1 1 1 -1 -1 inf]);
    y_loc = (repmat((detObj.y+y_size)',[1,6])) + y_size'*([-1 -1 1 1 -1 inf]);
    
    x_loc = x_loc';
    y_loc = y_loc';
    detObj.clusters_x_loc = x_loc(:);
    detObj.clusters_y_loc = y_loc(:);
    
    detObj.range = sqrt(detObj.y.*detObj.y + detObj.x.*detObj.x);
    detObj.doppler = (detObj.vy.*detObj.y + detObj.vx.*detObj.x)./detObj.range;
    
end
return

function [PA, idx] = getParkingAssistBins(bytevec, idx, tlvLen)
PARKING_ASSIST_BIN_SIZE_BYTES = 2;
detObj =[];
detObj.numObj = 0;
PA.x = [];
PA.y = [];
len_bytevec = length(bytevec);
if len_bytevec < idx +4; 
    idx = len_bytevec;
    return;
end
if tlvLen > 0
    %Get detected object descriptor
    word = [1 256]';
    detObj.numObj = sum(bytevec(idx+(1:2)) .* word);
    idx = idx + 2;
    xyzQFormat = 2^sum(bytevec(idx+(1:2)) .* word);
    idx = idx + 2;
    onebyXyzQFormat = 1/xyzQFormat;
    
    %Get detected array of detected objects
    if len_bytevec < idx + (1:detObj.numObj*PARKING_ASSIST_BIN_SIZE_BYTES); 
        idx = len_bytevec;
        return;
    end
    bytes = bytevec(idx+(1:detObj.numObj*PARKING_ASSIST_BIN_SIZE_BYTES ));
    idx = idx + detObj.numObj*PARKING_ASSIST_BIN_SIZE_BYTES;
    
    bytes = reshape(bytes, PARKING_ASSIST_BIN_SIZE_BYTES, detObj.numObj);
    range = ((bytes(1,:)+bytes(2,:)*256)*onebyXyzQFormat);
    
    range = fftshift(range);
    
    range = [range range(1)];
    
    xl = linspace(-1,1,detObj.numObj+2);
    yl = sqrt(1 - (xl.*xl));
    x_1 = range.*xl(1:end-1);
    x_2 = range.*xl(2:end);
    y_1 = range.*yl(1:end-1);
    y_2 = range.*yl(2:end);
    
    x = [x_1; x_2]; y = [y_1; y_2];
    
    PA.x = x(:);
    PA.y = y(:);
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



function guiMonitor = gui_initializer(Params, dim)
global MAX_NUM_CLUSTERS
global  load_parking_assist load_point_cloud_srr load_point_cloud_usrr load_clusters load_trackers view_range use_perspective_projection;

%% Setup the main figure
guiMonitor.detectedObjects = 1;
guiMonitor.stats = 0;
figHnd = figure(1);
clf(figHnd);
set(figHnd,'Name','Texas Instruments - AWR16xx Short Range Radar Demo ','NumberTitle','off')

warning off MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame
jframe=get(figHnd,'javaframe');
jIcon=javax.swing.ImageIcon('texas_instruments.gif');
jframe.setFigureIcon(jIcon);
% set(figHnd, 'MenuBar', 'none');
set(figHnd, 'Color', [0.8 0.8 0.8]);
set(figHnd, 'KeyPressFcn', @myKeyPressFcn);
set(figHnd, 'CloseRequestFcn', @myCloseRequestFcn);

% set(figHnd,'ResizeFcn',@Resize_clbk);

pause(0.00001);
set(jframe,'Maximized',1);
pause(0.00001);

%% Display chirp params
displayChirpParams(Params);
guiMonitor.figHnd = figHnd;

%% Initalize figures
% X, Y plot
guiMonitor.detectedObjectsFigHnd = subplot(3,5, [1 2 3 6 7 8 11 12 13]);
hold on
axis equal
axis([0 dim.max_dist_x 0 dim.max_dist_y])
xlabel('Distance along lateral axis (meters)');
ylabel('Distance along longitudinal axis (meters)');

% Populate the plots.
guiMonitor.detectedObjectsPlotHndA.Weak = plot(inf,inf,'g.', 'Marker', '.','MarkerSize',10); hold on;
guiMonitor.detectedObjectsPlotHndA.Mid = plot(inf,inf,'y.', 'Marker', '.','MarkerSize',10); hold on;
guiMonitor.detectedObjectsPlotHndA.Strong = plot(inf,inf,'w.', 'Marker', '.','MarkerSize',15); hold on;

guiMonitor.trackedObjPlotHnd = plot(inf,inf,'g.', 'Marker', 'd','MarkerSize',20);

guiMonitor.detectedObjectsPlotHndB.Weak = plot(inf,inf,'b.', 'Marker', '.','MarkerSize',10);
guiMonitor.detectedObjectsPlotHndB.Mid = plot(inf,inf,'c.', 'Marker', '.','MarkerSize',10);
guiMonitor.detectedObjectsPlotHndB.Strong = plot(inf,inf,'y.', 'Marker', '.','MarkerSize',15);

guiMonitor.clustersPlotHndB = plot(inf*ones(6*MAX_NUM_CLUSTERS,1),inf*ones(6*MAX_NUM_CLUSTERS,1),'c', 'LineWidth',2);
guiMonitor.parkingAssistRangeBinsHnd = plot(inf,inf,'g', 'LineWidth', 2, 'Color', [1 0.5 1]);
t = linspace(pi/6,5*pi/6,128);
plotRectGrid(dim.max_dist_y, dim.max_dist_x);
title('X-Y Scatter Plot')
set(gca,'Color',[0 0 0.5]);

%% R, Rd plot
guiMonitor.detectedObjectsRngDopFigHnd = subplot('Position',[0.618 0.625 0.375 0.3]);
hold off
hold on
set(gca,'Color',[0 0 0.5]);
axis([0 dim.max_dist_y -dim.max_vel dim.max_vel])
xlabel('Range (meters)');
ylabel('Doppler (m/s)');
title('Doppler-Range Plot');
set(gca,'Xcolor',[0.5 0.5 0.5]);
set(gca,'Ycolor',[0.5 0.5 0.5]);
guiMonitor.detectedObjectsRngDopPlotHndA.Strong = plot(inf,inf,'g.', 'Marker', '.','MarkerSize',10); hold on;
guiMonitor.detectedObjectsRngDopPlotHndA.Mid = plot(inf,inf,'g.', 'Marker', '.','MarkerSize',10); hold on;
guiMonitor.detectedObjectsRngDopPlotHndA.Weak = plot(inf,inf,'g.', 'Marker', '.','MarkerSize',10); hold on;

guiMonitor.trackedObjRngDop = plot(inf,inf,'g.', 'Marker', 'd','MarkerSize',13); hold on;

guiMonitor.detectedObjectsRngDopPlotHndB.Strong  = plot(inf,inf,'c.', 'Marker', '.','MarkerSize',10);
guiMonitor.detectedObjectsRngDopPlotHndB.Mid  = plot(inf,inf,'c.', 'Marker', '.','MarkerSize',10);
guiMonitor.detectedObjectsRngDopPlotHndB.Weak = plot(inf,inf,'c.', 'Marker', '.','MarkerSize',10);

clustersPlotHnd = plot(inf*ones(6*MAX_NUM_CLUSTERS,1),inf*ones(6*MAX_NUM_CLUSTERS,1),'c', 'LineWidth',2);
parkingAssistRangeBinsHnd = plot(inf,inf,'g', 'LineWidth', 2, 'Color', [1 0.5 1]);
plotRectGrid(dim.max_dist_y, dim.max_vel);

%% GUI filtering options
step = 0.15;
offset = 0.05;
halfStep = 0.2;
height = 0.075;
thickness = 0.3;
h_thickness = 0.5;
width = step;
chkbxwidth = step/5;

h_ui = uipanel('Parent',figHnd,'Title','Display Options.','FontSize',12, 'Units', 'normalized', 'Position',[0.618 0.10 0.274 0.1], 'BackgroundColor', [0.8 0.8 0.8]);

uicontrol('Parent',h_ui,'Units', 'normalized', 'Style','text','Position',[offset (height+h_thickness) width thickness],'String',        'Near View', 'BackgroundColor', [0.8 0.8 0.8]);
uicontrol('Parent',h_ui,'Units', 'normalized','Style','checkbox','Position',[(halfStep + offset) (height+h_thickness+0.002) chkbxwidth thickness],'Value',view_range,'Callback',@select_viewRange,'BackgroundColor', [0.8 0.8 0.8]);
uicontrol('Parent',h_ui,'Units', 'normalized', 'Style','text','Position',[(2*step + offset) (height+h_thickness) width thickness],'String',        'Parking ', 'BackgroundColor', [0.8 0.8 0.8]);
uicontrol('Parent',h_ui,'Units', 'normalized','Style','checkbox','Position',[(2*step + halfStep + offset) (height+h_thickness+0.002) chkbxwidth thickness],'Value',load_parking_assist,'Callback',@select_parking_assist,'BackgroundColor', [0.8 0.8 0.8]);
uicontrol('Parent',h_ui,'Units', 'normalized', 'Style','text','Position',[(4*step + offset) (height+h_thickness) width thickness],'String',        'USRR Cloud', 'BackgroundColor', [0.8 0.8 0.8]);
uicontrol('Parent',h_ui,'Units', 'normalized','Style','checkbox','Position',[(4*step + halfStep + offset) (height+h_thickness+0.002) chkbxwidth thickness],'Value',load_point_cloud_usrr,'Callback',@select_pointcloud_usrr,'BackgroundColor', [0.8 0.8 0.8]);
uicontrol('Parent',h_ui,'Units', 'normalized', 'Style','text','Position',[offset height width thickness],'String',        'SRR Cloud', 'BackgroundColor', [0.8 0.8 0.8]);
uicontrol('Parent',h_ui,'Units', 'normalized','Style','checkbox','Position',[(halfStep + offset) (height+0.002) chkbxwidth thickness],'Value',load_point_cloud_srr,'Callback',@select_pointcloud_srr,'BackgroundColor', [0.8 0.8 0.8]);
uicontrol('Parent',h_ui,'Units', 'normalized','Style','text','Position',[(2*step + offset) height width thickness],'String',        'Clusters','BackgroundColor', [0.8 0.8 0.8]);
uicontrol('Parent',h_ui,'Units', 'normalized','Style','checkbox','Position',[(2*step + halfStep + offset) (height+0.002) chkbxwidth thickness],'Value',load_clusters,'Callback',@select_clusters,'BackgroundColor', [0.8 0.8 0.8]);
uicontrol('Parent',h_ui,'Units', 'normalized','Style','text','Position',[(4*step + offset) height width thickness],'String',        'Trackers','BackgroundColor', [0.8 0.8 0.8]);
uicontrol('Parent',h_ui,'Units', 'normalized','Style','checkbox','Position',[(4*step  + halfStep + offset) (height+0.002) chkbxwidth thickness],'Value',load_trackers,'Callback',@select_trackers,'BackgroundColor', [0.8 0.8 0.8]);

if (guiMonitor.stats == 1)
    guiMonitor.statsFigHnd = subplot(3,3, 6);
    guiMonitor.statsPlotHnd = plot(zeros(100,3));
    figIdx = figIdx + 1;
    hold on;
    xlabel('frames');
    ylabel('% CPU Load');
    axis([0 100 0 100])
    title('Active and Interframe CPU Load')
    plot([0 0 0; 0 0 0])
    legend('Interframe', 'Active frame', 'GUI')
end

return

function select_pointcloud_srr(popup,event)
global load_point_cloud_srr;
load_point_cloud_srr = (popup.Value);
return

function select_pointcloud_usrr(popup,event)
global load_point_cloud_usrr;
load_point_cloud_usrr = (popup.Value);
return

function select_clusters(popup,event)
global load_clusters;
load_clusters = (popup.Value);
return

function select_trackers(popup,event)
global load_trackers;
load_trackers = (popup.Value);
return

function select_viewRange(popup,event)
global view_range;
view_range = (popup.Value);
return

function select_parking_assist(popup,event)
global load_parking_assist;
load_parking_assist = (popup.Value);

return
function select_perspective(popup, event)
global use_perspective_projection
use_perspective_projection = popup.Value;
return


function myKeyPressFcn(hObject, event)
global EXIT_KEY_PRESSED
if lower(event.Key) == 'q'
    EXIT_KEY_PRESSED  = 1;
end

return

function myCloseRequestFcn(hObject, event)
global EXIT_KEY_PRESSED
EXIT_KEY_PRESSED  = 1;

return
