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

function displayChirpParams(Params)
global hndChirpParamTable
global StatsInfo
global guiProcTime
    if hndChirpParamTable ~= 0
        delete(hndChirpParamTable);
        hndChirpParamTable = 0;
    end

    dat =  { '<HTML><b> Subframe #', 1; ...
            'Start Frequency (Ghz)',        Params(1).profileCfg.startFreq;...
            'Bandwidth (GHz)',              Params(1).profileCfg.freqSlopeConst * Params(1).profileCfg.numAdcSamples/Params(1).profileCfg.digOutSampleRate;...
            'Range resolution (m)',         Params(1).dataPath.rangeResolutionMeters;...
            'Velocity resolution (m/s)',    Params(1).dataPath.dopplerResolutionMps;...
            'Number of Tx (MIMO)',          Params(1).dataPath.numTxAnt;...
            '<HTML><b> Subframe #   ',      2; ...
            'Start Frequency (Ghz)',        Params(2).profileCfg.startFreq;...
            'Bandwidth (GHz)',              Params(2).profileCfg.freqSlopeConst * Params(2).profileCfg.numAdcSamples/Params(2).profileCfg.digOutSampleRate;...
            'Range resolution (m)',         Params(2).dataPath.rangeResolutionMeters;...
            'Velocity resolution (m/s)',    Params(2).dataPath.dopplerResolutionMps;...
            'Number of Tx (MIMO)',          Params(2).dataPath.numTxAnt;...
            'Subframe periodicity (msec)',  Params(2).frameCfg.framePeriodicity;...
            };
        
    columnname =   {'____________________Parameter (Units)________________', '____________Value____________'};
    columnformat = {'char', 'numeric'};
    
    t = uitable('Units','normalized','Position',...
                [0.618 0.22 0.274 0.34], 'Data', dat,... 
                'ColumnName', columnname,...
                'ColumnFormat', columnformat,...
                'ColumnWidth', 'auto',...
                'RowName',[]); 
            
    hndChirpParamTable = t;
return
