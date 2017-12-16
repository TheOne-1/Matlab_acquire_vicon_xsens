
%I do know how stupid this function is.
function result = getSensorStruct(num_ports, maxSamplesPreLocate)
if num_ports > 8
    error('\nBuddy our lab only have 8 Xsens sensors.')
end
oneSensorStruct(1:maxSamplesPreLocate) = struct('acc',zeros(1,3),'gyr',zeros(1,3),'mag',zeros(1,3));
switch num_ports
    case 1
        result = struct('IMU1', oneSensorStruct);
    case 2
        result = struct('IMU1', oneSensorStruct, 'IMU2', oneSensorStruct);
    case 3
        result = struct('IMU1', oneSensorStruct, 'IMU2', oneSensorStruct, ...
            'IMU3', oneSensorStruct);
    case 4
        result = struct('IMU1', oneSensorStruct, 'IMU2', oneSensorStruct, ...
            'IMU3', oneSensorStruct, 'IMU4', oneSensorStruct);
    case 5
        result = struct('IMU1', oneSensorStruct, 'IMU2', oneSensorStruct, ...
            'IMU3', oneSensorStruct, 'IMU4', oneSensorStruct, 'IMU5', oneSensorStruct);
    case 6
        result = struct('IMU1', oneSensorStruct, 'IMU2', oneSensorStruct, ...
            'IMU3', oneSensorStruct, 'IMU4', oneSensorStruct, 'IMU5', oneSensorStruct, ...
            'IMU6', oneSensorStruct);
    case 7
        result = struct('IMU1', oneSensorStruct, 'IMU2', oneSensorStruct, ...
            'IMU3', oneSensorStruct, 'IMU4', oneSensorStruct, 'IMU5', oneSensorStruct, ...
            'IMU6', oneSensorStruct, 'IMU7', oneSensorStruct);
    case 8
        result = struct('IMU1', oneSensorStruct, 'IMU2', oneSensorStruct, ...
            'IMU3', oneSensorStruct, 'IMU4', oneSensorStruct, 'IMU5', oneSensorStruct, ...
            'IMU6', oneSensorStruct, 'IMU7', oneSensorStruct, 'IMU8', oneSensorStruct);
end
end
