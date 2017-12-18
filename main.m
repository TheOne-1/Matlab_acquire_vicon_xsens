% 根据多次实验，一开始获得的vicon frame与xsens data之间不对齐，但是程序稳定后就对的比较齐了

function main
clear
clc
close all

routeRes = 'result.xlsx';
saveExcelData = false;
saveMatData = true;

% left_COP_offset = [-5.8; -13; -15.7];
% right_COP_offset = [1.04; -16.6; -14.2];

Fs = 100; % frequency of data
maxSamples = 1000; % set size of data to be recorded
maxSamplesPreLocate = round(maxSamples*1.05);

%% define variables


h = actxserver('xsensdeviceapi_com64.IXsensDeviceApi');
s =  h.XsScanner_scanPorts(0,100,true,true);		%检查所有com口
[num_ports, ~] = size(s);
device = uint64(zeros(1, num_ports));
deviceID = zeros(1, num_ports, 'uint32');
bandRate = zeros(1, num_ports, 'int32');
initializeXsens();


path(path,'D:\Software\DataStream SDK\Win64\MATLAB');
Client.LoadViconDataStreamSDK();
MyClient = Client();
initializeVicon();
subject_name = MyClient.GetSubjectName(1).SubjectName;
vicon_frame_rate = MyClient.GetFrameRate().FrameRateHz;
marker_num = MyClient.GetMarkerCount(subject_name, 1).MarkerCount;
marker_names = getMarkerNames();

iSample = ones(1, num_ports, 'int64'); % counter of current point
% data of xsens
gyr = zeros(maxSamplesPreLocate,3, num_ports);
acc = zeros(maxSamplesPreLocate,3, num_ports);
mag = zeros(maxSamplesPreLocate,3, num_ports);
euler_ls_xda = zeros(maxSamplesPreLocate,3, num_ports);
timeStamp = zeros(maxSamplesPreLocate, 1, num_ports);

% data of vicon
marker_pos = zeros(maxSamplesPreLocate, 3, marker_num);
unlabeled_marker_num = zeros(maxSamplesPreLocate, 1, 'unit8');
vicon_frame_number = zeros(maxSamplesPreLocate, 1, 'int64');
force_vector = zeros(maxSamplesPreLocate, 3, 2);
center_of_pressure = zeros(maxSamplesPreLocate, 3, 2);



%% event handling
h.registerevent({'onDataAvailable',@eventhandlerXsens});
h.setCallbackOption(h.XsComCallbackOptions_XSC_Packet, h.XsComCallbackOptions_XSC_None);
% show events using h.events and h.eventlisteners too see which are registerd;
fprintf( ' Reading data from devices... \n' );
while iSample(1) <= maxSamples + 5
    pause(.2)
end


%% save and show data
closeConnection();
result = getFormattedResult();

% % 用于确定相应的device
% for i = 1: num_ports
%     eval(['plot(result.IMU', num2str(i), '.acc(:,1));']);
%     hold on
% end
% legend(num2str(deviceID(1)), num2str(deviceID(2)), num2str(deviceID(3)), num2str(deviceID(4)), num2str(deviceID(5)));


% 用于比较vicon和xsens是否对齐
plot(result.vicon_data.vicon(1:maxSamples)); 


%% 每当Client收到Xsens的数据，都会回调该函数
    function eventhandlerXsens(varargin)
        
        % only action when new datapacket arrived
        dataPacket = varargin{3}{2};
        id = h.XsDataPacket_deviceId(dataPacket);
        
        % 仅在第一个device回调时调用vicon数据
        if id == deviceID(1)
            MyClient.GetFrame();
            vicon_frame_number(iSample(1)) = MyClient.GetFrameNumber().FrameNumber;
            unlabeled_marker_num(iSample(1)) =...
                MyClient.GetUnlabeledMarkerCount().MarkerCount;
            for iMarker = 1: marker_num
                % if the marker was absent at this frame, the translation will be [0, 0, 0]
                marker_pos(iSample(1), :, iMarker) = MyClient...
                    .GetMarkerGlobalTranslation(subject_name, marker_names(iMarker)).Translation;
            end
            force_vector(iSample(1), :, 1) = GetGlobalForceVector(1).ForceVector;
            force_vector(iSample(1), :, 2) = GetGlobalForceVector(2).ForceVector;
            center_of_pressure(iSample(1), :, 1) = GetGlobalCentreOfPressure(1).CentreOfPressure;
            center_of_pressure(iSample(1), :, 2) = GetGlobalCentreOfPressure(2).CentreOfPressure;
        end
        
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
                iSample(j) = iSample(j) + 1;
            end
        end
    end


    function initializeXsens()
        if isempty(s)
            fprintf( '\n Connection ports scanned - nothing found. \n' );
            return
        else
            % open port
            for iConnect = 1:num_ports
                deviceID(iConnect) = s{iConnect, 1};
                bandRate(iConnect) = s{iConnect, 4};
            end
            %cell s的第三列port信息实在难以取出来，每次都用s来调用
            fprintf('\n There are : %d', num_ports);
            fprintf(' devices\n');
        end
        
        %打开所有端口
        for iConnect = 1:num_ports
            h.XsControl_openPort(s{iConnect, 3}, bandRate(iConnect), 0, true);        %Open a communication channel on serial port with the given portname
            device(iConnect) = h.XsControl_device(deviceID(iConnect));        %Returns the XsDevice interface object associated with the supplied deviceId.
        end
        
        % 我也不知道showIntertialData能说明什么，反正官方
        showIntertialData = false(1,num_ports);
        for iConnect = 1:num_ports
            showIntertialData(iConnect) = h.XsDeviceId_isImu(deviceID(iConnect));     %Test if this device ID represents an IMU.
        end
        
        
        
        %% Enabling software Kalman filtering
        for iConnect = 1:num_ports
            h.XsDevice_setOptions(device(iConnect), h.XsOption_XSO_Orientation, 0);
        end
        
        %% Getting information about the devices
        for iConnect = 1:num_ports
            % put device in config mode
            h.XsDevice_gotoConfig(device(iConnect));
        end
        
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
        for iConnect = 1:num_ports
            %set to the new configuration
            h.XsDevice_setOutputConfiguration(device(iConnect),outputConfig_new);
        end
        
        
        %% Creating log file
        for iConnect = 1:num_ports
            filename = [cd '\logs\logfile_' dec2hex(deviceID(iConnect)) '.mtb'];
            h.XsDevice_createLogFile(device(iConnect),filename);
        end
        

        %% Entering measurement mode
        for iConnect = 1:num_ports
            h.XsDevice_gotoMeasurement(device(iConnect));
            % start recording
            h.XsDevice_startRecording(device(iConnect));
        end
        
    end

    %% Setup the vicon
    function initializeVicon()
        MyClient.Connect('localhost:801');
        MyClient.SetStreamMode( StreamMode.ClientPull);
        MyClient.GetFrame();
        
        MyClient.EnableMarkerData();
        MyClient.EnableDeviceData();
        
    end

    function marker_names = getMarkerNames()
        marker_names = cell(marker_num, 1);
        for iMarkerName = 1:marker_num
            marker_names(iMarkerName) = MyClient...
                .GetMarkerName(subject_name, iMarkerName).MarkerName;
        end
    end

    function closeConnection()
        %% Disconnect and dispose
        MyClient.Disconnect();
        % Unload the SDK
        
        for iDisConnect = 1: num_ports
            % Disabling channel
            h.XsDevice_stopRecording(device(iDisConnect));
        end
        
        for iDisConnect = 1: num_ports
            h.XsDevice_closeLogFile(device(iDisConnect));
            % close port and object
        end
        
        for iDisConnect = 1: num_ports
            % close port and object
            h.XsControl_closePort(s{iDisConnect, 3});
            h.XsControl_close();
        end
        delete(h); % release COM-object
        clear h;
    end




    function result = getFormattedResult()
        
        fprintf( ' ...done \n' );
        frame_total = vicon_frame_number(maxSamples) - vicon_frame_number(1);
        fprintf(' There are : %d vicon frames \n', frame_total);
        
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
        
        vicon_data = struct('frame_number', vicon_frame_number, 'subject_name', subject_name,...
            'marker_names', marker_names, 'marker_pos', marker_pos, 'force_vector', force_vector,...
            'CoP', center_of_pressure, 'unlabeled_marker_number', unlabeled_marker_num);
        result.vicon_data = vicon_data;
        
        if saveMatData
            save result result
        end
    end
end




