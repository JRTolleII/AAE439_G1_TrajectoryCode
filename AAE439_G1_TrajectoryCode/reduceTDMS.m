function [dataStruc] = reduceTDMS(TDMSfilename,ZeroType,ZeroPressure,ZeroStruc)
% Written by: Kevin Dille 03/14/2022
%
% This function reduces any TDMS file and returns a structure containing
% all channels in the TDMS files. User can also provide local pressure to
% override any zeroing targets of "14.7 psi" to whatever the provided
% ambient pressure is.
%
% INPUTS:
%   TDMSfilename:   Filename of the TDMS file you wish to reduce
%   ZeroType:       Selection for how you'd like to handle data zeroing:
%                       0: Only reduces the raw data, without using the
%                           zeroing correction within the TDMS file.
%                       1: Zeros the data using the zeroing correction
%                           contained within the input TDMS file.
%                       2: Zeros the data by correcting against another
%                           reduced TDMS file structure. Format of this
%                           structure must follow the same structure as
%                           dataStruc. This structure could be generated by
%                           a previous reduceTDMS call with ZeroType=0.
%   ZeroPressure:   Local ambient pressure (psi) to override any PT 
%                   targeting "14.7" psi. Leave as "nan" or "14.7" to not
%                   change what number to zero too.
%   ZeroStruc       Structure variable of format:
%                           ZeroStruc.(channelName).Value = dummy
%                       Where dummy can be an array or numeric constant.
%                       The average of the entirety of dummy will be used
%                       as the zeroing correction for the channel matching
%                       (channelName).
%
% OUTPUTS:
%   dataStruc:      Structure variable with subfields for every channel
%                   contained within the input TDMS file.

%% Check Inputs
if ZeroType == 2
    if ~(nargin == 4)
        fprintf('Not the correct number of inputs.\n')
        fprintf('Expecting 4 inputs in order to zero the TDMS data against another TDMS file.\n\n')
        return
    end
end


%% Load and Parse Data from File
% Verify file exists in directory:
if  exist(TDMSfilename,'file') == 0
    error('TDMS file does not exist in current directory.')
end



%% Load and parse data
% TDMS non-proprietary format load data and process
%% Load Data from AI10K
[ConvertedData,ConvertVer,ChanNames,GroupNames,ci]=convertTDMS(0,TDMSfilename);
chanNum = max(size(ChanNames{1}));

% Parse Header Indices
tempProp = {ConvertedData.Data.MeasuredData(3).Property.Name};
idxName  = find( strcmp(tempProp, 'Channel Name'));
idxSlope = find( strcmp(tempProp, 'Slope'));
idxOffset= find( strcmp(tempProp, 'Offset'));
idxZero  = find( strcmp(tempProp, 'Zeroing Correction'));
idxUnits = find( strcmp(tempProp, 'Unit'));
idxTCType = find( strcmp(tempProp, 'TC Type'));

% Parse data and header info into chanData structure
for i = 1:chanNum
    chanData(i).rawdata = ConvertedData.Data.MeasuredData(i+2).Data;
    chanData(i).property = ConvertedData.Data.MeasuredData(i+2).Property;
    
    % Parse Channel Data
    chanData(i).name     = chanData(i).property(idxName).Value;
    chanData(i).slope    = chanData(i).property(idxSlope).Value;
    chanData(i).offset   = chanData(i).property(idxOffset).Value;
    chanData(i).zero     = chanData(i).property(idxZero).Value;
    chanData(i).units    = chanData(i).property(idxUnits).Value;
    chanData(i).TCType   = chanData(i).property(idxTCType).Value;
end
nameSplit = strsplit(ChanNames{1}{1}, {'(',')'});
scan_rate = str2num(cell2mat(strsplit(nameSplit{2}, ' Hz')));
samps = ConvertedData.Data.MeasuredData(3).Total_Samples;
dt = 1/scan_rate;

%% Save into Structure
dataStruc.time.Name = 'Time';
dataStruc.time.Value = linspace(0,samps*dt,samps)';
dataStruc.time.Units = 'seconds';

% Save Data to Output Structure
for i = 1:max(size(chanData))
    tempName = chanData(i).name;
    tempModName = strrep(tempName,'-','_');
    tempModName = strrep(tempModName,' ','');
    if contains(tempModName,'10k_lc')
        tempModName = strrep(tempModName,'10k_lc','lc_10k');
    end
    dataStruc.(tempModName).Name = tempName;
    
    
    
    if chanData(i).units == 'f' % Channel is a thermocouple and is linear
        dataStruc.(tempModName).Value = chanData(i).rawdata(1:samps);
    else % Channel is a linear output
        if ZeroType == 0 % Returns raw signal
            zeroTemp = 0;
        elseif ZeroType == 1 % Zeros to corretion within the same TDMS file
            zeroTemp = chanData(i).zero;
            
            % Correct Zero if zeroing to updated local pressure
            if ~isnan(ZeroPressure)
                if ZeroPressure ~= 14.7
                    if chanData(i).property(4).Value == 14.7
                        zeroTemp = zeroTemp + (ZeroPressure - chanData(i).property(4).Value);
                    end
                end
            end
            
        elseif ZeroType == 2 % Zeros by averaging channel from another TDMS file
            zeroTarget = chanData(i).property(4).Value; %Zeroing Target
            if isnan(zeroTarget)
                % Do Not correct this channel if the zeroTarget is NaN
                zeroTemp = 0;
            else
                meanTemp = mean(ZeroStruc.(tempModName).Value);
                zeroTemp = zeroTarget - meanTemp;
                
                % Correct Zero if zeroing to updated local pressure
                if ~isnan(ZeroPressure)
                    if ZeroPressure ~= 14.7
                        if chanData(i).property(4).Value == 14.7
                            zeroTemp = zeroTemp + (ZeroPressure - chanData(i).property(4).Value);
                        end
                    end
                end
                
            end
        end
        
        
        
        % Compute Signal in Engineering Units:
        dataStruc.(tempModName).Value = ...
            chanData(i).rawdata(1:samps)*chanData(i).slope + chanData(i).offset + zeroTemp;
    end
    dataStruc.(tempModName).Units = chanData(i).units;
end

end