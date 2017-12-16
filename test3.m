function test3
%% Scanning connection ports
h = actxserver('xsensdeviceapi_com64.IXsensDeviceApi');

s =  h.XsScanner_scanPorts(0,100,true,true);		%检查所有com口
if isempty(s)
    fprintf( '\n Connection ports scanned - nothing found. \n' );
    return
else
    fprintf( '\n Connection ports scanned \n' );
    % open port
    [num_ports, ~] = size(s);
%     deviceID = uint32(zeros(1, num_ports));
%     bandRate = zeros(1, num_ports);
    for i = 1:num_ports
        deviceID(i) = s{i, 1};
        bandRate(i) = s{i,4};
    end
    %cell s的第三列port信息实在难以取出来，每次都用s来调用
    fprintf('\n There are : %d', num_ports);
    fprintf(' devices\n');
end

device = uint64(zeros(1, num_ports));
%打开所有端口
for i = 1:num_ports
    h.XsControl_openPort(s{i, 3}, bandRate(i), 0, true);        %Open a communication channel on serial port with the given portname
    device(i) = h.XsControl_device(deviceID(i));        %Returns the XsDevice interface object associated with the supplied deviceId. 
end

% check device id for interial data
showIntertialData = false(1,num_ports);
for i = 1:num_ports
    showIntertialData(i) = h.XsDeviceId_isImu(deviceID(i));     %Test if this device ID represents an IMU. 
end



%% Enabling software Kalman filtering
% When software filtering is enabled it is possible to obtain
% orientation data without setting orientation as an output (in that
% case calibrated data or SDI and magnetic field data are necessary). If
% orientation is set as an output, enabling software filtering is not
% necessary.
for i = 1:num_ports
    h.XsDevice_setOptions(device(i), h.XsOption_XSO_Orientation, 0);
    % Compute orientation, the orientation is only computed in one stream.
    % If not specified the system will decide: when reading from file it will
    % use XSO_OrientationInBufferedStream, otherwise XSO_OrientationInLiveStream
%     fprintf( '\n Software Filtering - set\n' );
end


%% Getting information about the devices

for i = 1:num_ports
    % put device in config mode
    h.XsDevice_gotoConfig(device(i));
end

Fs = 100; % frequency of data
% first column is the data identifier, second column is the frequency
% frequency of zero means that this data is send with each pakket.
if (showIntertialData)
    outputConfig_new = {h.XsDataIdentifier_XDI_PacketCounter,0;   % sample counter
        h.XsDataIdentifier_XDI_SampleTimeFine,0;   % sample time fine
        h.XsDataIdentifier_XDI_DeltaV,Fs; % dv, 100Hz
        h.XsDataIdentifier_XDI_DeltaQ,Fs; % dq, 100Hz
        h.XsDataIdentifier_XDI_MagneticField,Fs}; % magnetic field, 100Hz
else
    outputConfig_new = {h.XsDataIdentifier_XDI_PacketCounter,0;   % Packet counter, increments every packet.
        h.XsDataIdentifier_XDI_SampleTimeFine,0;   % sample time fine
        h.XsDataIdentifier_XDI_Quaternion,Fs; % orientation in quats, 100Hz
        h.XsDataIdentifier_XDI_DeltaV,Fs; % dv, 100Hz
        h.XsDataIdentifier_XDI_DeltaQ,Fs; % dq, 100Hz
        h.XsDataIdentifier_XDI_MagneticField,Fs; % magnetic field, 100Hz
        h.XsDataIdentifier_XDI_StatusWord,0};  % status word (contains clipping)
end
for i = 1:num_ports
    %set to the new configuration
    h.XsDevice_setOutputConfiguration(device(i),outputConfig_new);
end


%% Creating log file
for i = 1:num_ports
    filename = [cd '\logfile_' dec2hex(deviceID(i)) '.mtb'];
    h.XsDevice_createLogFile(device(i),filename);
end

%% preallocation
maxSamples = 1000; % set size of data to be recorded
maxSamplesPreLocate = round(maxSamples*1.05); % set size of data prelocated, because of eventhandling the
% measured data can be more than maxSamples

oneSensor(1:maxSamplesPreLocate) = struct('acc',zeros(1,3), 'gyr',zeros(1,3),...
    'mag',zeros(1,3), 'euler_ls_xda', zeros(1,3), 'timeStamp', zeros(1, 'int32'));
result = struct('IMU1', oneSensor);
oneClip(1:maxSamplesPreLocate) = struct('acc',zeros(1,3, 'logical'),...
     'gyr',zeros(1,3, 'logical'), 'mag',zeros(1,3, 'logical'));
clip = struct('IMU1', oneClip);
for i = 2:num_ports
    % create a structure which contains the data of all the sensors
    eval(['[result(:).IMU', num2str(i), '] = oneSensor;']);
    % create a clip information structure
    eval(['[clip(:).IMU', num2str(i), '] = oneClip;']);
end

% q_ls_xda = zeros(maxSamplesPreLocate,4, num_ports);
% euler_ls_xda = zeros(maxSamplesPreLocate,3, num_ports);
% sampleCounter = NaN(maxSamplesPreLocate,1, num_ports);

iSample = ones(1, num_ports, 'int64'); % counter of current point
skipFactor = 5; % for plotting

%% Entering measurement mode
for i = 1:num_ports
    % fprintf( '\n Activate measurement mode \n' );
    h.XsDevice_gotoMeasurement(device(i));
    % start recording
    h.XsDevice_startRecording(device(i));
    % fprintf('\n Logfile: %s created, start recording.\n',filename);
end

%% event handling
% register eventhandler回调函数
h.registerevent({'onDataAvailable',@eventhandlerXsens});
h.setCallbackOption(h.XsComCallbackOptions_XSC_Packet, h.XsComCallbackOptions_XSC_None);
% show events using h.events and h.eventlisteners too see which are registerd;

%% receive data
t1 = now();
fprintf( '\n Reading data from devices... \n' );
while iSample(1) < maxSamplesPreLocate * 5
    pause(.2)
end
t2 = now();
fprintf(num2str(t2 - t1)/n);

save('result.mat', 'result');
save('clip.mat', 'clip');

%% event handling, 获得数据最核心的函数
    function eventhandlerXsens(varargin)
        
        %         onBufferedDataAvailable
        % only action when new datapacket arrived
        dataPacket = varargin{3}{2};
        id = h.XsDataPacket_deviceId(dataPacket);
        for j = 1: num_ports
            if id == deviceID(j)
                % get the time stamp
                eval(['result.IMU', num2str(j), '(', num2str(iSample(j)),...
                    ').timeStamp = round(h.XsDataPacket_sampleTimeFine(dataPacket)/100);']);
                % get the sdi data, return the strapdown integration data component of a data item.
                sdiData = cell2mat(h.XsDataPacket_sdiData(dataPacket));
                % get acc data
                eval(['result.IMU', num2str(j), '(', num2str(iSample(j)),...
                    ').acc = sdiData(5:7) * Fs;']);
%                 n = norm(sdiData(2:4));     % 取模
%                 % get gyr data
%                 if n>eps        % 设置阈值eps，小于该阈值则视为0。然而天降eps，完全不知道哪来的
%                     eval(['result.IMU', num2str(j), '(', num2str(iSample(j)),...
%                         ').gyr = (2*asin(n)*Fs)*(sdiData(2:4)/n);']);
%                 else
%                     eval(['result.IMU', num2str(j), '(', num2str(iSample(j)),...
%                         ').gyr = zeros(3,1);']);
%                 end
%                 % get mag data
%                 if h.XsDataPacket_containsCalibratedMagneticField(dataPacket) &&...
%                         (~h.XsDataPacket_containsCalibratedData(dataPacket) ||...
%                         h.XsDataPacket_containsSdiData(dataPacket))
%                     % overwrite sdi mag data;
%                     eval(['result.IMU', num2str(j), '(', num2str(iSample(j)),...
%                         ').mag = cell2mat(h.XsDataPacket_calibratedMagneticField(dataPacket));']);
%                 end
%                 % get orientation data, represented as eular angle
%                 if h.XsDataPacket_containsOrientation(dataPacket)
%                     % getting quaternions, return the orientation component of a data item as a euler angles
%                     eval(['result.IMU', num2str(j), '(', num2str(iSample(j)),...
%                         ').euler_ls_xda = cell2mat(h.XsDataPacket_orientationEuler(dataPacket,h.XsDataIdentifier_XDI_CoordSysEnu));']);
%                 end
%                 
%                 clipData = 0;
%                 if h.XsDataPacket_containsStatus(dataPacket)
%                     % The status component of a data item.
%                     clipData = bitget(h.XsDataPacket_status(dataPacket),9:17)';
%                 end
%                 if any(clipData)
%                     clipFlags = logical(clipData);
%                     eval(['clip.IMU', num2str(j), '(', num2str(iSample(j)),...
%                         ').acc = clipFlags(1:3);']);
%                     eval(['clip.IMU', num2str(j), '(', num2str(iSample(j)),...
%                         ').gyr = clipFlags(4:6);']);
%                     eval(['clip.IMU', num2str(j), '(', num2str(iSample(j)),...
%                         ').mag = clipFlags(7:9);']);
%                 end
                iSample(j) = iSample(j) + 1;
            end
        end
    end
end





