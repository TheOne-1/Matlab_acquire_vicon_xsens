function main
clear
clc
close all

routeRes = 'result.xlsx';
saveExcelData = false;
saveMatData = true;

% left_COP_offset = [-5.8; -13; -15.7];
% right_COP_offset = [1.04; -16.6; -14.2];

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
    deviceID = zeros(1, num_ports, 'uint32');
    bandRate = zeros(1, num_ports, 'int32');
    for i = 1:num_ports
        deviceID(i) = s{i, 1};
        bandRate(i) = s{i, 4};
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

% 我也不知道showIntertialData能说明什么，反正官方
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
    filename = [cd '\logs\logfile_' dec2hex(deviceID(i)) '.mtb'];
    h.XsDevice_createLogFile(device(i),filename);
end

%% preallocation
maxSamples = 60000; % set size of data to be recorded
maxSamplesPreLocate = round(maxSamples*1.05); % set size of data prelocated, because of eventhandling the
% measured data can be more than maxSamples

gyr = zeros(maxSamplesPreLocate,3, num_ports);
acc = zeros(maxSamplesPreLocate,3, num_ports);
mag = zeros(maxSamplesPreLocate,3, num_ports);
euler_ls_xda = zeros(maxSamplesPreLocate,3, num_ports);
timeStamp = zeros(maxSamplesPreLocate,1, num_ports);

iSample = ones(1, num_ports, 'int64'); % counter of current point
%% Entering measurement mode
for i = 1:num_ports
    % fprintf( '\n Activate measurement mode \n' );
    h.XsDevice_gotoMeasurement(device(i));
    % start recording
    h.XsDevice_startRecording(device(i));
    % fprintf('\n Logfile: %s created, start recording.\n',filename);
end

%% Setup the vicon
path(path,'D:\Software\DataStream SDK\Win64\MATLAB');
% Load the SDK
Client.LoadViconDataStreamSDK();
% Connect as a client to the Vicon server
MyClient = Client();
MyClient.Connect('localhost:801');
MyClient.SetStreamMode( StreamMode.ClientPull);

% MyClient = Client();
% MyClient.Connect( 'localhost' );

%% event handling
% register eventhandler回调函数
h.registerevent({'onDataAvailable',@eventhandlerXsens});
h.setCallbackOption(h.XsComCallbackOptions_XSC_Packet, h.XsComCallbackOptions_XSC_None);

MyClient.GetFrame();
vicon_frame_number(1) = MyClient.GetFrameNumber().FrameNumber;
% show events using h.events and h.eventlisteners too see which are registerd;

%% receive data
fprintf( ' Reading data from devices... \n' );
while iSample(1) <= maxSamples + 5
    pause(.2)
end


%% Disconnect and dispose
MyClient.Disconnect();
% Unload the SDK
% Client.UnloadViconDataStreamSDK();


for i = 1: num_ports
    % Disabling channel
    h.XsDevice_stopRecording(device(i));
end

for i = 1: num_ports
    h.XsDevice_closeLogFile(device(i));
    % close port and object
end

for i = 1: num_ports
    % close port and object
    h.XsControl_closePort(s{i, 3});
    h.XsControl_close();
end

delete(h); % release COM-object
clear h;

%% save and show data

fprintf( ' ...done \n' );
frame_total = vicon_frame_number(2) - vicon_frame_number(1);
fprintf(' There are : %d vicon frames \n', frame_total);
% fprintf('vicon frames');

result = getFormattedResult();
if saveMatData
    save result result
end
% % 用于确定相应的device
% for i = 1: num_ports
%     eval(['plot(result.IMU', num2str(i), '.acc(:,1));']);
%     hold on
% end
% legend(num2str(deviceID(1)), num2str(deviceID(2)), num2str(deviceID(3)), num2str(deviceID(4)), num2str(deviceID(5)));

%% event handling, 获得数据最核心的函数
    function eventhandlerXsens(varargin)
        
        % 检测尾部是否也对齐了
        if iSample(1) == maxSamples - 1
            MyClient.GetFrame();
            vicon_frame_number(2) = MyClient.GetFrameNumber().FrameNumber;
        end
        
        %         onBufferedDataAvailable
        % only action when new datapacket arrived
        dataPacket = varargin{3}{2};
        id = h.XsDataPacket_deviceId(dataPacket);
        for j = 1: num_ports
            if id == deviceID(j)
                % get the time stamp
                timeStamp(iSample(j), 1, j) = round(h.XsDataPacket_sampleTimeFine(dataPacket)/100);
                % get the sdi data, return the strapdown integration data component of a data item.
                sdiData = cell2mat(h.XsDataPacket_sdiData(dataPacket));
                % get acc data
                acc(iSample(j), :, j) = sdiData(5:7) * Fs;
                n = norm(sdiData(2:4));     % 取模
                % get gyr data
                if n>eps        % 设置阈值eps，小于该阈值则视为0。然而天降eps，完全不知道哪来的
                    gyr(iSample(j), :, j) = (2*asin(n)*Fs)*(sdiData(2:4)/n);
                else
                    gyr(iSample(j), :, j) = zeros(3,1);
                end
                % get mag data
                if h.XsDataPacket_containsCalibratedMagneticField(dataPacket) &&...
                        (~h.XsDataPacket_containsCalibratedData(dataPacket) ||...
                        h.XsDataPacket_containsSdiData(dataPacket))
                    % overwrite sdi mag data;
                    mag(iSample(j), :, j) = cell2mat(h.XsDataPacket_calibratedMagneticField(dataPacket));
                end
                % get orientation data, represented as eular angle
                if h.XsDataPacket_containsOrientation(dataPacket)
                    % getting quaternions, return the orientation component of a data item as a euler angles
                    euler_ls_xda(iSample(j), :, j)  = cell2mat(h.XsDataPacket_orientationEuler(dataPacket,h.XsDataIdentifier_XDI_CoordSysEnu));
                end
                
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

    function result = getFormattedResult
        if saveExcelData
            for k = 1: num_ports
                xlswrite(routeRes, {'Time Stamp'}, num2str(deviceID(k)), 'A1');
                xlswrite(routeRes, timeStamp(:, :, k), num2str(deviceID(k)), 'A2');
                xlswrite(routeRes, {'Accelaration'}, num2str(deviceID(k)), 'C1');
                xlswrite(routeRes, acc(:, :, k), num2str(deviceID(k)), 'B2');
                xlswrite(routeRes, {'Gyroscope'}, num2str(deviceID(k)), 'F1');
                xlswrite(routeRes, gyr(:, :, k), num2str(deviceID(k)), 'E2');
                xlswrite(routeRes, {'Magnetometer'}, num2str(deviceID(k)), 'I1');
                xlswrite(routeRes, mag(:, :, k), num2str(deviceID(k)), 'H2');
                xlswrite(routeRes, {'Euler Angle'}, num2str(deviceID(k)), 'L1');
                xlswrite(routeRes, euler_ls_xda(:, :, k), num2str(deviceID(k)), 'K2');
            end
        end
        % I do know how stupid the following code is, but using function
        % eval is even more stupid. Please fold this function when using.
        if num_ports > 8
            error('Buddy, our lab only have 8 Xsens sensors');
        end
        switch num_ports
            case 1
                IMU1 = struct('timeStamp', timeStamp(:, :, 1), 'acc', acc(:, :, 1),...
                    'gyr', gyr(:, :, 1), 'mag', mag(:, :, 1), 'euler', euler_ls_xda(:, :, 1));
                result = struct('IMU1', IMU1);
            case 2
                IMU1 = struct('timeStamp', timeStamp(:, :, 1), 'acc', acc(:, :, 1),...
                    'gyr', gyr(:, :, 1), 'mag', mag(:, :, 1), 'euler', euler_ls_xda(:, :, 1));
                IMU2 = struct('timeStamp', timeStamp(:, :, 2), 'acc', acc(:, :, 2),...
                    'gyr', gyr(:, :, 2), 'mag', mag(:, :, 2), 'euler', euler_ls_xda(:, :, 2));
                result = struct('IMU1', IMU1, 'IMU2', IMU2);
            case 3
                IMU1 = struct('timeStamp', timeStamp(:, :, 1), 'acc', acc(:, :, 1),...
                    'gyr', gyr(:, :, 1), 'mag', mag(:, :, 1), 'euler', euler_ls_xda(:, :, 1));
                IMU2 = struct('timeStamp', timeStamp(:, :, 2), 'acc', acc(:, :, 2),...
                    'gyr', gyr(:, :, 2), 'mag', mag(:, :, 2), 'euler', euler_ls_xda(:, :, 2));
                IMU3 = struct('timeStamp', timeStamp(:, :, 3), 'acc', acc(:, :, 3),...
                    'gyr', gyr(:, :, 3), 'mag', mag(:, :, 3), 'euler', euler_ls_xda(:, :, 3));
                result = struct('IMU1', IMU1, 'IMU2', IMU2, 'IMU3', IMU3);
            case 4
                IMU1 = struct('timeStamp', timeStamp(:, :, 1), 'acc', acc(:, :, 1),...
                    'gyr', gyr(:, :, 1), 'mag', mag(:, :, 1), 'euler', euler_ls_xda(:, :, 1));
                IMU2 = struct('timeStamp', timeStamp(:, :, 2), 'acc', acc(:, :, 2),...
                    'gyr', gyr(:, :, 2), 'mag', mag(:, :, 2), 'euler', euler_ls_xda(:, :, 2));
                IMU3 = struct('timeStamp', timeStamp(:, :, 3), 'acc', acc(:, :, 3),...
                    'gyr', gyr(:, :, 3), 'mag', mag(:, :, 3), 'euler', euler_ls_xda(:, :, 3));
                IMU4 = struct('timeStamp', timeStamp(:, :, 4), 'acc', acc(:, :, 4),...
                    'gyr', gyr(:, :, 4), 'mag', mag(:, :, 4), 'euler', euler_ls_xda(:, :, 4));
                result = struct('IMU1', IMU1, 'IMU2', IMU2, 'IMU3', IMU3, 'IMU4', IMU4);
            case 5
                IMU1 = struct('timeStamp', timeStamp(:, :, 1), 'acc', acc(:, :, 1),...
                    'gyr', gyr(:, :, 1), 'mag', mag(:, :, 1), 'euler', euler_ls_xda(:, :, 1));
                IMU2 = struct('timeStamp', timeStamp(:, :, 2), 'acc', acc(:, :, 2),...
                    'gyr', gyr(:, :, 2), 'mag', mag(:, :, 2), 'euler', euler_ls_xda(:, :, 2));
                IMU3 = struct('timeStamp', timeStamp(:, :, 3), 'acc', acc(:, :, 3),...
                    'gyr', gyr(:, :, 3), 'mag', mag(:, :, 3), 'euler', euler_ls_xda(:, :, 3));
                IMU4 = struct('timeStamp', timeStamp(:, :, 4), 'acc', acc(:, :, 4),...
                    'gyr', gyr(:, :, 4), 'mag', mag(:, :, 4), 'euler', euler_ls_xda(:, :, 4));
                IMU5 = struct('timeStamp', timeStamp(:, :, 5), 'acc', acc(:, :, 5),...
                    'gyr', gyr(:, :, 5), 'mag', mag(:, :, 5), 'euler', euler_ls_xda(:, :, 5));
                result = struct('IMU1', IMU1, 'IMU2', IMU2, 'IMU3', IMU3, 'IMU4', IMU4,...
                    'IMU5', IMU5);
            case 6
                IMU1 = struct('timeStamp', timeStamp(:, :, 1), 'acc', acc(:, :, 1),...
                    'gyr', gyr(:, :, 1), 'mag', mag(:, :, 1), 'euler', euler_ls_xda(:, :, 1));
                IMU2 = struct('timeStamp', timeStamp(:, :, 2), 'acc', acc(:, :, 2),...
                    'gyr', gyr(:, :, 2), 'mag', mag(:, :, 2), 'euler', euler_ls_xda(:, :, 2));
                IMU3 = struct('timeStamp', timeStamp(:, :, 3), 'acc', acc(:, :, 3),...
                    'gyr', gyr(:, :, 3), 'mag', mag(:, :, 3), 'euler', euler_ls_xda(:, :, 3));
                IMU4 = struct('timeStamp', timeStamp(:, :, 4), 'acc', acc(:, :, 4),...
                    'gyr', gyr(:, :, 4), 'mag', mag(:, :, 4), 'euler', euler_ls_xda(:, :, 4));
                IMU5 = struct('timeStamp', timeStamp(:, :, 5), 'acc', acc(:, :, 5),...
                    'gyr', gyr(:, :, 5), 'mag', mag(:, :, 5), 'euler', euler_ls_xda(:, :, 5));
                IMU6 = struct('timeStamp', timeStamp(:, :, 6), 'acc', acc(:, :, 6),...
                    'gyr', gyr(:, :, 6), 'mag', mag(:, :, 6), 'euler', euler_ls_xda(:, :, 6));
                result = struct('IMU1', IMU1, 'IMU2', IMU2, 'IMU3', IMU3, 'IMU4', IMU4,...
                    'IMU5', IMU5, 'IMU6', IMU6);
            case 7
                IMU1 = struct('timeStamp', timeStamp(:, :, 1), 'acc', acc(:, :, 1),...
                    'gyr', gyr(:, :, 1), 'mag', mag(:, :, 1), 'euler', euler_ls_xda(:, :, 1));
                IMU2 = struct('timeStamp', timeStamp(:, :, 2), 'acc', acc(:, :, 2),...
                    'gyr', gyr(:, :, 2), 'mag', mag(:, :, 2), 'euler', euler_ls_xda(:, :, 2));
                IMU3 = struct('timeStamp', timeStamp(:, :, 3), 'acc', acc(:, :, 3),...
                    'gyr', gyr(:, :, 3), 'mag', mag(:, :, 3), 'euler', euler_ls_xda(:, :, 3));
                IMU4 = struct('timeStamp', timeStamp(:, :, 4), 'acc', acc(:, :, 4),...
                    'gyr', gyr(:, :, 4), 'mag', mag(:, :, 4), 'euler', euler_ls_xda(:, :, 4));
                IMU5 = struct('timeStamp', timeStamp(:, :, 5), 'acc', acc(:, :, 5),...
                    'gyr', gyr(:, :, 5), 'mag', mag(:, :, 5), 'euler', euler_ls_xda(:, :, 5));
                IMU6 = struct('timeStamp', timeStamp(:, :, 6), 'acc', acc(:, :, 6),...
                    'gyr', gyr(:, :, 6), 'mag', mag(:, :, 6), 'euler', euler_ls_xda(:, :, 6));
                IMU7 = struct('timeStamp', timeStamp(:, :, 7), 'acc', acc(:, :, 7),...
                    'gyr', gyr(:, :, 7), 'mag', mag(:, :, 7), 'euler', euler_ls_xda(:, :, 7));
                result = struct('IMU1', IMU1, 'IMU2', IMU2, 'IMU3', IMU3, 'IMU4', IMU4,...
                    'IMU5', IMU5, 'IMU6', IMU6, 'IMU7', IMU7);
            case 8
                IMU1 = struct('timeStamp', timeStamp(:, :, 1), 'acc', acc(:, :, 1),...
                    'gyr', gyr(:, :, 1), 'mag', mag(:, :, 1), 'euler', euler_ls_xda(:, :, 1));
                IMU2 = struct('timeStamp', timeStamp(:, :, 2), 'acc', acc(:, :, 2),...
                    'gyr', gyr(:, :, 2), 'mag', mag(:, :, 2), 'euler', euler_ls_xda(:, :, 2));
                IMU3 = struct('timeStamp', timeStamp(:, :, 3), 'acc', acc(:, :, 3),...
                    'gyr', gyr(:, :, 3), 'mag', mag(:, :, 3), 'euler', euler_ls_xda(:, :, 3));
                IMU4 = struct('timeStamp', timeStamp(:, :, 4), 'acc', acc(:, :, 4),...
                    'gyr', gyr(:, :, 4), 'mag', mag(:, :, 4), 'euler', euler_ls_xda(:, :, 4));
                IMU5 = struct('timeStamp', timeStamp(:, :, 5), 'acc', acc(:, :, 5),...
                    'gyr', gyr(:, :, 5), 'mag', mag(:, :, 5), 'euler', euler_ls_xda(:, :, 5));
                IMU6 = struct('timeStamp', timeStamp(:, :, 6), 'acc', acc(:, :, 6),...
                    'gyr', gyr(:, :, 6), 'mag', mag(:, :, 6), 'euler', euler_ls_xda(:, :, 6));
                IMU7 = struct('timeStamp', timeStamp(:, :, 7), 'acc', acc(:, :, 7),...
                    'gyr', gyr(:, :, 7), 'mag', mag(:, :, 7), 'euler', euler_ls_xda(:, :, 7));
                IMU8 = struct('timeStamp', timeStamp(:, :, 8), 'acc', acc(:, :, 8),...
                    'gyr', gyr(:, :, 8), 'mag', mag(:, :, 8), 'euler', euler_ls_xda(:, :, 8));
                result = struct('IMU1', IMU1, 'IMU2', IMU2, 'IMU3', IMU3, 'IMU4', IMU4,...
                    'IMU5', IMU5, 'IMU6', IMU6, 'IMU7', IMU7, 'IMU8', IMU8);
        end
    end
end




