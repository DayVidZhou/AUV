filename = 'testyaw1.txt';
yaw_data = csvread(filename, 1, 0); % Skips first row
t = yaw_data(:,1);
ax = yaw_data(:,2);
ay = yaw_data(:,3);
az = yaw_data(:,4);
gx = yaw_data(:,5);
gy = yaw_data(:,6);
gz = yaw_data(:,7);

figure
title('Dynamic Response to 100 [ms] Yaw input Signal for 1.5 ms')
subplot(3,1,1)
plot(t,gz)
xlabel('Time [ms]'), ylabel('gz [deg/sec]')
subplot(3,1,2)
plot(t,ax)
xlabel('Time [ms]'), ylabel('ax [m/sec^2]')
subplot(3,1,3)
plot(t,ay)
xlabel('Time [ms]'), ylabel('ay [m/sec^2]')
% start yaw at 3.5 seconds

%%
filename = 'testheave1.txt';
heave_data = csvread(filename, 1, 0); % Skips first row
t = heave_data(:,1)./1000;
ax = heave_data(:,2);
ay = heave_data(:,3);
az = heave_data(:,4);
gx = heave_data(:,5);
gy = heave_data(:,6);
gz = heave_data(:,7);
n = length(az);
vz = zeros(1, n);
az_offset = mean(az(1:400));
az = az - az_offset;

for i = 2:n
    vz(i) = vz(i - 1) + (t(i)-t(i-1))*az(i);
end

figure
title('Dynamic Response to 100 [ms] Heave input Signal for 1.5 s')
subplot(4,1,1)
plot(t,az)
xlabel('Time [ms]'), ylabel('Heave [m/sec^2]'), xticks([0:1:25])
subplot(4,1,2)
plot(t,ax)
xlabel('Time [ms]'), ylabel('Surge [m/sec^2]')
subplot(4,1,3)
plot(t,gy)
xlabel('Time [ms]'), ylabel('Pitch [deg/sec]')
subplot(4,1,4)
plot(t,vz)
xlabel('Time [ms]'), ylabel('vz [deg/sec]'), xticks([0:1:25])

%%
filename = 'testsurge1.txt';
surge_data = csvread(filename, 1, 0); % Skips first row
t = surge_data(:,1)./1000;
ax = surge_data(:,2);
ay = surge_data(:,3);
az = surge_data(:,4);
gx = surge_data(:,5);
gy = surge_data(:,6);
gz = surge_data(:,7);

n = length(az);
vx = zeros(1, n);
ax_offset = mean(ax(1:500));
ax = ax - ax_offset;

for i = 2:n
    vx(i) = vx(i - 1) + (t(i)-t(i-1))*ax(i);
end


figure
title('Dynamic Response to 100 [ms] Surge input Signal for 1.5 s')
subplot(4,1,1)
plot(t,ax)
xlabel('Time [ms]'), ylabel('Surge [m/sec^2]'), xticks([0:1:25])
subplot(4,1,2)
plot(t,az)
xlabel('Time [ms]'), ylabel('Heave [m/sec^2]')
subplot(4,1,3)
plot(t,gy)
xlabel('Time [ms]'), ylabel('Pitch [deg/sec]')
subplot(4,1,4)
plot(t,vx)
xlabel('Time [ms]'), ylabel('vx [deg/sec]'), xticks([0:1:25])