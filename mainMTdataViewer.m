% 	Copyright (c) 2003-2017 Xsens Technologies B.V. or subsidiaries worldwide.
%	All rights reserved.

%	Redistribution and use in source and binary forms, with or without modification,
%	are permitted provided that the following conditions are met:

%	1.	Redistributions of source code must retain the above copyright notice,
%		this list of conditions and the following disclaimer.

%	2.	Redistributions in binary form must reproduce the above copyright notice,
%		this list of conditions and the following disclaimer in the documentation
%		and/or other materials provided with the distribution.

%	3.	Neither the names of the copyright holders nor the names of their contributors
%		may be used to endorse or promote products derived from this software without
%		specific prior written permission.

%	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
%	EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
%	MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
%	THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
%	SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
%	OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
%	HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
%	TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
%	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


function mainMTdataViewer
    %%------- HELP
    %
    % This script allows the user to understand the step-wise procedure to get data from MKIV devices. 
    %
    % The code is divided into two parts:
    %
    % 1) Set-up of the system
    %
    % 2) Event handler of the MT.
    %
    %%-------- IMPORTANT NOTES
    %
    % - For the code to work properly, make sure the code folder is your current directory in Matlab.
    %
    % - This code supports both 32 and 64 bits Matlab version.
    %
    % - The code requires xsensdeviceapi_com.dll to be registered in the Windows
    %   register (this is done automatically during the Xsens MT SDK installation)
    %
    
    %% Launching activex server
    try
        switch computer
        case 'PCWIN'
            h = actxserver('xsensdeviceapi_com32.IXsensDeviceApi');
        case 'PCWIN64'
            h = actxserver('xsensdeviceapi_com64.IXsensDeviceApi');
        otherwise
            error('CMT:os','Unsupported OS');
        end
    catch e
        fprintf('\n Please reinstall MT SDK or check manual,\n Xsens Device Api is not found.\n')
        rethrow(e);
    end
    fprintf( '\n ActiveXsens server - activated \n' );
	
	version = h.XsControl_version;
    fprintf(' XDA version: %.0f.%.0f.%.0f\n',version{1:3})
    if length(version)>3
        fprintf(' XDA build: %.0f%s\n',version{4:5})
    end
    
    %% Scanning connection ports
    s =  h.XsScanner_scanPorts(0,100,true,true);
    if isempty(s)
        fprintf( '\n Connection ports scanned - nothing found. \n' );
        return
    else
        fprintf( '\n Connection ports scanned \n' );
        % open port
        deviceID = s{1,1};
        portS = s{1,3};
        baudRate = s{1,4};
        [num_ports, ~] = size(s);
        if num_ports > 1
            fprintf('\n More than one device port found, this script only uses the first one listed: %s\n', dec2hex(deviceID));
        end
        h.XsControl_openPort(portS,baudRate,0,true);
    end

    device = h.XsControl_device(deviceID);
    
    if ~(h.XsDeviceId_isMtMk4(deviceID) || h.XsDeviceId_isFmt_X000(deviceID))
        fprintf('\nERROR: Only MtMk4 devices supported in this example\n');
        return
    end
    
    % check device id for interial data
    showIntertialData = h.XsDeviceId_isImu(deviceID);

    %% Enabling software Kalman filtering
    % When software filtering is enabled it is possible to obtain
    % orientation data without setting orientation as an output (in that
    % case calibrated data or SDI and magnetic field data are necessary). If
    % orientation is set as an output, enabling software filtering is not
    % necessary.

    h.XsDevice_setOptions(device, h.XsOption_XSO_Orientation, 0);
    fprintf( '\n Software Filtering - set\n' );

    %% Getting information about the devices
    % put device in config mode
    h.XsDevice_gotoConfig(device);

    % h.XsDevice_setOutputConfiguration(device,outputConfig)
    % set output mode for this script:
    outputConfig = h.XsDevice_outputConfiguration(device);
    % first column is the data identifier, second column is the frequency
    % frequency of zero means that this data is send with each pakket.
    Fs = 100; % frequency of data
    
    if (showIntertialData)
        outputConfig_new = {h.XsDataIdentifier_XDI_PacketCounter,0;   % sample counter
                            h.XsDataIdentifier_XDI_SampleTimeFine,0;   % sample time fine
                            h.XsDataIdentifier_XDI_DeltaV,Fs; % dv, 100Hz
                            h.XsDataIdentifier_XDI_DeltaQ,Fs; % dq, 100Hz
                            h.XsDataIdentifier_XDI_MagneticField,Fs}; % magnetic field, 100Hz
    else    
        outputConfig_new = {h.XsDataIdentifier_XDI_PacketCounter,0;   % sample counter
                            h.XsDataIdentifier_XDI_SampleTimeFine,0;   % sample time fine
                            h.XsDataIdentifier_XDI_Quaternion,Fs; % orientation in quats, 100Hz
                            h.XsDataIdentifier_XDI_DeltaV,Fs; % dv, 100Hz
                            h.XsDataIdentifier_XDI_DeltaQ,Fs; % dq, 100Hz
                            h.XsDataIdentifier_XDI_MagneticField,Fs; % magnetic field, 100Hz
                            h.XsDataIdentifier_XDI_StatusWord,0};  % status word (contains clipping)
    %outputConfig_new(:,1) = cellfun(@(x) hex2dec(x),outputConfig_new(:,1),'UniformOutput',false);
    end
    h.XsDevice_setOutputConfiguration(device,outputConfig_new);
    
    %% Creating log file
    filename = [cd '\logfile_' dec2hex(deviceID) '.mtb'];
    h.XsDevice_createLogFile(device,filename); 

    %% preallocation
    maxSamples = 1000; % set size of data to be recorded
    maxSamplesPreLocate = round(maxSamples*1.05); % set size of data prelocated, because of eventhandling the 
                                                  % measured data can be more than maxSamples
                                                  
    gyr = zeros(maxSamplesPreLocate,3);
    acc = zeros(maxSamplesPreLocate,3);
    mag = zeros(maxSamplesPreLocate,3);
    % clip 表示需要裁剪的
    clip(1:maxSamplesPreLocate) = struct('acc',zeros(1,3),'gyr',zeros(1,3),'mag',zeros(1,3));
    q_ls_xda = zeros(maxSamplesPreLocate,4);
    euler_ls_xda = zeros(maxSamplesPreLocate,3);
    sampleCounter = NaN(maxSamplesPreLocate,1);
    timeStamp = zeros(maxSamplesPreLocate,1);
    
    iSample = 1; % counter
    skipFactor = 5; % for plotting
    %% Initialize plotting data 
    % (plots are update by events)
    devId = num2str(dec2hex(deviceID));
    
    if (showIntertialData)
        [e_f e_ax e] = plotInertialAcc(sampleCounter, acc,...                  % signal
                               strcat('Acceleration - Device SN-',devId), ... % title
                               ['r' 'g' 'b']);                                  % color
        [q_f q_ax q] = plotInertialAngVel(sampleCounter, gyr,...                  % signal
                               strcat('Angular Velocity - Device SN-', devId),... % title
                               ['r' 'g' 'b']);                          % color
        [m_f m_ax m] = plotInertialMag(sampleCounter, mag,...                  % signal
                               strcat('Magnetic Field - Device SN-', devId),... % title
                               ['r' 'g' 'b']);                          % color                   
    else    
        [e_f e_ax e] = plotSubplot(sampleCounter, euler_ls_xda,...                  % signal (Euler angles)
                               strcat('Euler Angles - Device SN-',devId), {'x-Roll', 'y-Pitch', 'z-Yaw'},... % title
                               ['r' 'g' 'b']);                                  % color
        [q_f q_ax q] = plotSubplotQuat(sampleCounter, q_ls_xda,...                  % signal (quaternion
                                   strcat('Quaternion - Device SN-', devId),... % title
                                   ['k' 'r' 'g' 'b']);                          % color
    end

    % connect variables to lines
    if (showIntertialData)
        for i=1:3
            set(q(i),'xDataSource','timeStamp(gyr(:,1)~=0)-timeStamp(1)',... 
                     'yDataSource',['gyr(gyr(:,1)~=0,' num2str(i) ')']);
        end
        
        for i=1:3
            set(e(i),'xDataSource','timeStamp(acc(:,1)~=0)-timeStamp(1)',...
                     'yDataSource',['acc(acc(:,1)~=0,' num2str(i) ')']);
        end
        
        for i=1:3
            set(m(i),'xDataSource','timeStamp(mag(:,1)~=0)-timeStamp(1)',...
                     'yDataSource',['mag(mag(:,1)~=0,' num2str(i) ')']);
        end 
    else
        for i=1:4
            set(q(i),'xDataSource','timeStamp(q_ls_xda(:,1)~=0)-timeStamp(1)',... 
                     'yDataSource',['q_ls_xda(q_ls_xda(:,1)~=0,' num2str(i) ')']);
        end
        
        for i=1:3
            set(e(i),'xDataSource','timeStamp(euler_ls_xda(:,1)~=0)-timeStamp(1)',...
                     'yDataSource',['euler_ls_xda(euler_ls_xda(:,1)~=0,' num2str(i) ')']);
        end 
    end

    % set xlim
    xlims = [0 maxSamples/100];
    set(q_ax,'xlim',xlims);
    set(e_ax,'xlim',xlims);
    
    if (showIntertialData)
         set(m_ax,'xlim',xlims);
    end

    %% Entering measurement mode
    fprintf( '\n Activate measurement mode \n' );
    h.XsDevice_gotoMeasurement(device);
    % start recording
    h.XsDevice_startRecording(device);
    fprintf('\n Logfile: %s created, start recording.\n',filename);
    %% event handling
    % register eventhandler
    %Called when new data has been received from a device or read from a file. When processing on PC is enabled, 
    %this callback occurs after processing has been done and so the packet will contain the processing output. 
    h.registerevent({'onLiveDataAvailable',@eventhandlerXsens});
    %enable live packet callback
    h.setCallbackOption(h.XsComCallbackOptions_XSC_LivePacket, h.XsComCallbackOptions_XSC_None);

    % show events using h.events and h.eventlisteners too see which are registerd;
    %% receive data
    fprintf( '\n Reading data from devices... \n' );
    while  iSample < maxSamples
        % wait untill maxSamples are arrived
        pause(.2)
    end

    fprintf( '\n ...done \n' );
    if iSample>maxSamplesPreLocate,fprintf( '\n more samples measured than prelocated! \n' );end
    % cutting preallocating exceeding data
    gyr(iSample:end,:,:) =[];
    acc(iSample:end,:,:) =[];
    mag(iSample:end,:,:) =[];
    clip(iSample:end,:) =[];
    q_ls_xda(iSample:end,:,:) =[];
    euler_ls_xda(iSample:end,:,:) =[];



    %% Disabling channel
    h.XsDevice_stopRecording(device);
    h.XsDevice_gotoConfig(device);
    h.XsDevice_closeLogFile(device);

    %% close port and object
    h.XsControl_closePort(portS);
    h.XsControl_close();
    
    delete(h); % release COM-object
    clear h;

    %% saving data
    savefig('MTMK4_test');
    
    if (showIntertialData)
        savefig(e_f, strcat('Acceleration - Device SN-',devId,'_MTi_results'));
        savefig(q_f, strcat('Angular Velocity - Device SN-',devId,'_MTi_results'));
        savefig(m_f, strcat('Magnetic Field - Device SN-',devId,'_MTi_results'));
    else 
        savefig(e_f, strcat('Euler Angles - Device SN-',devId,'_MTw_results'));
        savefig(q_f, strcat('Quaternion - Device SN-',devId,'_MTw_results'));
    end
%% event handling
    function eventhandlerXsens(varargin)
        % only action when new datapacket arrived
        dataPacket = varargin{3}{2};
        if dataPacket && iSample < maxSamples
            % check if sample counter is in packet
            if h.XsDataPacket_containsPacketCounter(dataPacket)
                % sample counter
                sampleCounter(iSample) = h.XsDataPacket_packetCounter(dataPacket);      %画图时用来做横坐标 
            end
            if h.XsDataPacket_containsSampleTimeFine(dataPacket)
                % sample time
                timeStamp(iSample:end) = h.XsDataPacket_sampleTimeFine(dataPacket)/1e4;      %Return the fine sample time of a packet. 
            end
            if h.XsDataPacket_containsSdiData(dataPacket)       %Check if data item contains strapdown integration data. 
                % getting SDI data (1:4) = dq, (5:7) = dv
                sdiData = cell2mat(h.XsDataPacket_sdiData(dataPacket));         %Return the strapdown integration data component of a data item. 
                n = norm(sdiData(2:4));
                if n>eps        % 设置阈值eps，小于该阈值则视为0。然而天降eps，完全不知道哪来的
                    gyr(iSample,:) = (2*asin(n)*Fs)*(sdiData(2:4)/n)* 180/pi; %rad2deg
                else
                    gyr(iSample,:) = zeros(3,1);
                end
                acc(iSample,:) = sdiData(5:7)'*Fs;
            elseif h.XsDataPacket_containsCalibratedData(dataPacket)
                % calibrated data (acc, gyr, mag all three channels)
                calData = cell2mat(h.XsDataPacket_calibratedData(dataPacket));

                acc(iSample,:) = calData(1:3);
                gyr(iSample,:) = calData(4:6);
                mag(iSample,:) = calData(7:9);
            end

            if h.XsDataPacket_containsCalibratedMagneticField(dataPacket) && (~h.XsDataPacket_containsCalibratedData(dataPacket) || h.XsDataPacket_containsSdiData(dataPacket))
                % overwrite sdi mag data;
                mag(iSample,:) = cell2mat(h.XsDataPacket_calibratedMagneticField(dataPacket));
            end


            if h.XsDataPacket_containsOrientation(dataPacket)
                % getting quaternions, return the orientation component of a data item as a quaternion. 
                q_ls_xda(iSample,:) = cell2mat(h.XsDataPacket_orientationQuaternion(dataPacket,h.XsDataIdentifier_XDI_CoordSysEnu))';
                % getting Euler angles, return the orientation component of a data item as a euler angles
                euler_ls_xda(iSample,:) = cell2mat(h.XsDataPacket_orientationEuler(dataPacket,h.XsDataIdentifier_XDI_CoordSysEnu))';
            end
            
            clipData = 0;
            if h.XsDataPacket_containsStatus(dataPacket)
                % The status component of a data item.
                clipData = bitget(h.XsDataPacket_status(dataPacket),9:17)';
            end
            if any(clipData)
                clipFlags = logical(clipData);
                clip(iSample).acc = clipFlags(1:3);
                clip(iSample).gyr = clipFlags(4:6);
                clip(iSample).mag = clipFlags(7:9);
            end

            h.liveDataPacketHandled(varargin{3}{1}, dataPacket);

            % plot
            if mod(iSample,skipFactor)==0
                % refresh plots
                if exist('m','var')
                    l = [q e m];
                else
                    l = [q e];
                end
                    
                for line = l
                    refreshdata(line,'caller');
                end
            end
        end
        iSample = iSample + 1;
    end
end
