close all
load result.mat
len = 1000;
acc1 = zeros(len, 3);
acc2 = zeros(len, 3);
for i = 1: len
   acc1(i, :) = result.IMU1(i).acc;
end
plot(acc1(:, 1))
hold on

for i = 1: len
   acc2(i, :) = result.IMU2(i).acc;
end
plot(acc2(:, 1))