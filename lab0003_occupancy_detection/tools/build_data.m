%
% Copyright (c) 2018 Texas Instruments Incorporated
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

% -------------------------------------------------------------------------
% Build the training data set for two zones using multiple measurement scenes
% Dependency: calc_features.m
%
% -------------------------------------------------------------------------
function [X, y] = build_data(dataListFile, windowLen)
    if (nargin ~= 2)
        error('Incorrect number of arguments\n');
    end

    global zone_noise_floor;
    global zone_leakage;
    global zone_occ_power;

    zone_noise_floor = zeros(1, 2);
    zone_leakage = zeros(1, 2);
    zone_occ_power = zeros(1, 2);

    winLen = str2double(windowLen);

    % zone definition file
%    zoneDefFile = 'zone_def.cfg';

    % parse the data file names and associated tags
%    fileName    = 'data_list.txt';
    [filenameAndTags, zoneDef, fullZone] = parse_dataList(dataListFile);

    fliplrSet   = [0];

    %% build training data
    [X, y] = loopThroughFiles(filenameAndTags, fliplrSet, zoneDef, winLen);

    if (fullZone == 0) %if the both zone occupied case is not included, create it
        fprintf('Creating both zone occupied case...\n');

        for flagReverse = [0,1]
            [X_, y_] = calc_features(filenameAndTags{1,1}, 99, ...
                                     zoneDef, winLen, [], flagReverse, 0);
            X = [X; X_];
            y = [y; y_];
        end
    end

    % save the output
    matFilename = sprintf('data_winLen=%d.mat', winLen);
    save(matFilename, 'X', 'y', 'winLen', 'filenameAndTags');
    fprintf('Zone noise floor: %f %f\n', zone_noise_floor(1), zone_noise_floor(2));
    fprintf('Zone leakage lvl: %f %f\n', zone_leakage(1), zone_leakage(2));
    fprintf('Zone occupid pwr: %f %f\n', zone_occ_power(1), zone_occ_power(2));
    fprintf('%s saved\n', matFilename);

end


% -------------------------------------------------------------------------
function [X, y] = loopThroughFiles(filenameAndTags, fliplrSet, zoneDef, winLen)

    X = [];
    y = [];

    nFiles = size(filenameAndTags,1);

    for flagReverse = [0,1]
        for flagFliplr = fliplrSet
            for iFile = 1:nFiles
                [X_, y_] = calc_features(filenameAndTags{iFile,1}, ...
                                         bi2de(filenameAndTags{iFile,2}), ...
                                         zoneDef, winLen, [], flagReverse, flagFliplr);
                X = [X; X_];
                y = [y; y_];
            end
        end
    end
end


% -------------------------------------------------------------------------
function [filenameAndTags, zoneDef, foundFull] = parse_dataList(fileName)
    % Parse the list of file names and associated tags

    if exist(fileName, 'file')==2
        fileId  = fopen(fileName, 'r');
    else
        error('Cannot open <%s>\n', fileName);
    end

    pat_2values = '^(?<name>\S+([.])?\w+)\s+(?<value1>[+-]?([0-9]*[.])?[0-9]+)\s+(?<value2>[+-]?([0-9]*[.])?[0-9]+).*';
    line        = fgetl(fileId);
    lineIdx     = 0;
    foundFull   = 0;

    while ischar(line)
        v = regexp(line, pat_2values, 'names');

        if ~isempty(v)
            switch v.name

                case 'zoneDef'
                % parse zone definitions
                zoneDef = parse_zoneDef(line);

                otherwise
                    lineIdx = lineIdx + 1;
                    filenameAndTags{lineIdx,1} = v.name;
                    filenameAndTags{lineIdx,2} = [str2num(v.value1), str2num(v.value2)];

                    class = bi2de(filenameAndTags{lineIdx,2});
                    if (class == 3)
                        foundFull = 1; %Found the both zone occupied case
                    end
            end
        end

        line = fgetl(fileId);
    end

    if (lineIdx == 0)
        error('No files found in <%s>\n', fileName);
    end

    fclose(fileId);

end

%------------------------------------------------------------------------------
% Calculate the feature set from measurement data (range-azimuth heatmap) for zone-based occupancy detection
%
% X: feature data matrix, (numFrames-winLen+1) x nFeatures
% y: classification output for each zone, (numFrames-winLen+1) x nZones
%
% Features: for 2 zones,
% - avgPwr_zone1
% - avgPwr_zone2
% - pwrRatio_zone1
% - pwrRatio_zone2
% - xcorrPwr_zone12
%
%------------------------------------------------------------------------------
function [X, y, zonePwrdB, zonePairs, zone, rangeAzimuth, Params] = calc_features(dataFilename,...
                                     classTag, zoneDef, winLen, framesIdx, flagReverse, flagFliplr)
    if nargin<4, winLen         = 12; end % window length
    if nargin<5, framesIdx      = []; end
    if nargin<6, flagReverse    = 0;  end
    if nargin<7, flagFliplr     = 0;  end

    global zone_noise_floor;
    global zone_leakage;
    global zone_occ_power;

    persistent zonePwrOcc;
    persistent zonePwrOccdB;

    reduce_factor = 1.0;
    flagRev = int8(flagReverse) + 1; % use as an array index

    %% read the data .mat file
    [rangeAzimuth, Params] = read_dataFile(dataFilename);
    rangeVal_       = Params.rangeVal_m;
    azimuthVal      = Params.azimuthVal_degree;
    numRangeBins    = Params.numRangeBins;
    numFrames       = Params.numFrames;
    nZones          = length(zoneDef);

    % rearrange data depending the input arguments
    if flagReverse
        rangeAzimuth = flipdim(rangeAzimuth, 3);
    end

    if flagFliplr
        rangeAzimuth = flipdim(rangeAzimuth, 2);

        % update classTag accordingly: below is only for two zones located side-by-side alone the cross-range
        if ~isempty(classTag)
            classTag = bi2de(fliplr(de2bi(classTag, nZones)));
        end
    end

    % truncate data when 'framesIdx' is specified
    if ~isempty(framesIdx)
        rangeAzimuth = rangeAzimuth(:, :, framesIdx(1):framesIdx(2));
    end

    % range indices of interest: 'nEdge' range bins at the both edges are not considered
    nEdge           = 0;
    rgIdx           = (1+nEdge:numRangeBins-nEdge);
    rangeVal        = rangeVal_(rgIdx);

    for zIdx = 1:nZones
        zone(zIdx)  = define_zone(rangeVal, azimuthVal, zoneDef{zIdx});
    end

    %% zone-power in each frame: raw data for features

    if (classTag < 99)
        zonePwr    = zeros(numFrames, nZones);
        zonePwrdB  = zeros(numFrames, nZones);
     
        for zIdx = 1:nZones
            %this is (range+2) x (azimuth+2) x numFrames for a zone = 1 cell pad on all sides
            rangeAzimuth_rgAzGated  = rangeAzimuth(zone(zIdx).rgIdx, zone(zIdx).azIdx, :);
     
            % lengths are padded sizes
            rangeLen = size(rangeAzimuth_rgAzGated, 1);
            azimLen  = size(rangeAzimuth_rgAzGated, 2);
     
            %this is numFrames x 2 zones (one zone filled per iteration)
            zonePwr(:, zIdx)   = get_5x5(rangeAzimuth_rgAzGated, rangeLen, azimLen, numFrames);
     
            if (classTag > 0) %a zone is occupied
              tmp = (zonePwr(:, zIdx) - zone_noise_floor(zIdx)) / reduce_factor;
              zonePwr(:, zIdx) = tmp + zone_noise_floor(zIdx);
            end
     
            zonePwrdB(:, zIdx) = 10*log10(zonePwr(:, zIdx)); % in dB
        end
    else
        zonePwr    = zonePwrOcc(:, :, flagRev);
        zonePwrdB  = zonePwrOccdB(:, :, flagRev);
    end

    %% calculate features
    avgPwr      = zeros(numFrames - winLen + 1, nZones);
    avgPwrdB    = zeros(numFrames - winLen + 1, nZones);
    pwrRatio    = zeros(numFrames - winLen + 1, nZones);
    pwrRatiodB  = zeros(numFrames - winLen + 1, nZones);

    zonePairs   = nchoosek(1:nZones, 2);
    nPairs      = size(zonePairs, 1);
    xcorrCoeff  = zeros(numFrames - winLen + 1, nPairs);

    for iFrame = winLen:numFrames

        % indices for moving window
        winIdx = ((iFrame-winLen+1):iFrame);

        for iZone = 1:nZones
            % average zone power (linear)
            avgPwr(iFrame-winLen+1, iZone)   = mean(zonePwr(winIdx, iZone));

            % average zone power (dB)
            avgPwrdB(iFrame-winLen+1, iZone) = 10*log10(avgPwr(iFrame-winLen+1, iZone));
        end

        % power ratio
        pwrRatio(iFrame-winLen+1,:)   = avgPwr(iFrame-winLen+1, :) / sum(avgPwr(iFrame-winLen+1, :), 2);
        pwrRatiodB(iFrame-winLen+1,:) = 10*log10(pwrRatio(iFrame-winLen+1, :));

        % correlation coefficient between pairs
        corrCoeff = corrcoef(zonePwrdB(winIdx, :));

        % xcorr between zone powers
        for iPair = 1:nPairs
             zoneA = zonePairs(iPair, 1);
             zoneB = zonePairs(iPair, 2);
             xcorrCoeff(iFrame - winLen + 1, iPair) = corrCoeff(zoneA, zoneB);
        end
    end

    if (classTag == 0) %noise floor case (zones empty)
        zone_noise_floor(1) = mean(zonePwr(:, 1)); %left
        zone_noise_floor(2) = mean(zonePwr(:, 2)); %right
    elseif (classTag == 1) %[0,1] right occupied
        zone_leakage(1) = mean(zonePwr(:, 1));
        zone_occ_power(2) = mean(zonePwr(:, 2));
        zonePwrOcc(:, 2, flagRev) = zonePwr(:, 2);        %save for creation of state 3
        zonePwrOccdB(:, 2, flagRev) = zonePwrdB(:, 2);
    elseif (classTag == 2) %[1,0] left occupied
        zone_occ_power(1) = mean(zonePwr(:, 1));
        zone_leakage(2) = mean(zonePwr(:, 2));
        zonePwrOcc(:, 1, flagRev) = zonePwr(:, 1);        %save for creation of state 3
        zonePwrOccdB(:, 1, flagRev) = zonePwrdB(:, 1);
    end

    %% populate X and y
    X = [avgPwrdB, pwrRatiodB, xcorrCoeff];

    if isempty(classTag)
        y = [];
    else
        y = repmat(classTag(:).', size(X,1), 1);
    end
end


% -------------------------------------------------------------------------
% Find the maximum 5x5 region within a zone, for each frame. Inputs:
% zone_rngAz: zone data sized with 1 cell boundary on all sides
% rangeLen: range dimension of zone plus 2 (for the 1 cell boundary)
% azimLen: azimuth dimension of zone plus 2 (for the 1 cell boundary)
function zonePower = get_5x5(zone_rngAz, rangeLen, azimLen, numFrames)

    for frmIdx = 1:numFrames
        %find the largest 3x3 area in this frame
        max = 0;
        
        for rangeIdx = 2:rangeLen-3
            for azimIdx = 2:azimLen-3
                temp = sum(sum(zone_rngAz(rangeIdx:rangeIdx+2, azimIdx:azimIdx+2, frmIdx)));

                if (temp > max)
                    maxRidx = rangeIdx;
                    maxAidx = azimIdx;
                    max = temp;
                end
            end
        end

        %Now, the indexes point to the inner 3x3 of the 5x5 that we want. Find the average of it.
        zonePower(frmIdx) = mean(mean(zone_rngAz(maxRidx-1:maxRidx+3, maxAidx-1:maxAidx+3, frmIdx), 1), 2);
    end
end


% -------------------------------------------------------------------------
function [rangeAzimuth, params] = read_dataFile(dataFilename)

    %% load the data .mat file
    if exist(dataFilename)
        load(dataFilename, 'rangeAzimuth_log', 'Params', 'numFrames');
    else
        error('%s does not exist.', dataFilename);
    end

    % reshape data: rangeAzimuth(rangeIdx, azimuthIdx, frameIdx)
    numRangeBins       = Params.dataPath.numRangeBins;
    numAngleBins       = Params.dataPath.numAngleBins;
    rangeAzimuth       = reshape(rangeAzimuth_log, numAngleBins, numRangeBins, numFrames);
    rangeAzimuth       = permute(rangeAzimuth, [2, 1, 3]);

    % form necessary params
    params.numRangeBins         = numRangeBins;
    params.numAngleBins         = numAngleBins;
    params.numFrames            = numFrames;
    params.rangeVal_m           = Params.dataPath.rangeResolutionMeters*[0:numRangeBins-1];
    params.azimuthVal_radian    = (-numAngleBins/2:numAngleBins/2-1)/numAngleBins*(60/90)*pi;
    params.azimuthVal_degree    = (-numAngleBins/2:numAngleBins/2-1)/numAngleBins*(60/90)*180;

end


%------------------------------------------------------------------------------
function zone = define_zone(rgVal, azVal, def)
    % zoneDef: range_start range_length azimuth_start azimuth_length.
    % range_start and azimuth_start index starts from zero
    % NOTE: This function now pads the zone indexes (size) by 1 cell on all sides.

    zone.def    = def;
    zone.rgIdx  = (zone.def(1):zone.def(1)+zone.def(2)+1);
    zone.azIdx  = (zone.def(3):zone.def(3)+zone.def(4)+1);
    zone.rect   = [azVal(zone.azIdx(1)), rgVal(zone.rgIdx(1)), ...
                   azVal(zone.azIdx(end))-azVal(zone.azIdx(1)), rgVal(zone.rgIdx(end))-rgVal(zone.rgIdx(1))];

    % generates a set of points for the boundary of a zone
    zone.boundary = gen_zonePoints(zone.rect);

end


%------------------------------------------------------------------------------
% Parse the zone definitions
%
function  zoneDef = parse_zoneDef(fileLine)

    zoneDef = [];

    pat_9values = '^(?<name>\w+)\s+(?<value1>[+-]?([0-9]*[.])?[0-9]+)\s+(?<value2>[+-]?([0-9]*[.])?[0-9]+)\s+(?<value3>[+-]?([0-9]*[.])?[0-9]+)\s+(?<value4>[+-]?([0-9]*[.])?[0-9]+)\s+(?<value5>[+-]?([0-9]*[.])?[0-9]+)\s+(?<value6>[+-]?([0-9]*[.])?[0-9]+)\s+(?<value7>[+-]?([0-9]*[.])?[0-9]+)\s+(?<value8>[+-]?([0-9]*[.])?[0-9]+)\s+(?<value9>[+-]?([0-9]*[.])?[0-9]+).*';

    if ischar(fileLine)
        v = regexp(fileLine, pat_9values, 'names');

        if ~isempty(v)

            numzones = str2num(v.value1);
            if (numzones ~= 2)
                error('Error - number of zones must be 2!\n');
                exit;
            end
            switch v.name

                case 'zoneDef'
                    zoneDef{1} = [str2num(v.value2), str2num(v.value3), str2num(v.value4), str2num(v.value5)];
                    zoneDef{2} = [str2num(v.value6), str2num(v.value7), str2num(v.value8), str2num(v.value9)];

                otherwise
                    fprintf('%s ingored\n', v.name);
            end
        end
    end
end


%------------------------------------------------------------------------------
% generates a set of points for the boundary of a zone
% to be overlaid on the polar-coordinate plot
function zonePoints = gen_zonePoints(zoneRect)

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

end
