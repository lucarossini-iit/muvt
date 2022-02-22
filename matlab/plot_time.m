clear all
close all
clc

load('optimizer_stats_good.mat')

%% plot opt time
figure(1)
hold on 

% cut vertices
opt_time = opt_time(find(time > 72));
time = time(find(time > 72));

time = time - ones(1,length(time))*time(1);
opt_time = opt_time(find(time < 100));
time = time(find(time < 100));

% filter
d1 = designfilt('lowpassiir','FilterOrder',12, ...
    'HalfPowerFrequency',0.15,'DesignMethod','butter');
opt_time_filt = filtfilt(d1, opt_time);

% plot
plot(time, opt_time, 'LineWidth', 1, 'Color', '#9FBBD6')
plot(time, opt_time_filt, 'LineWidth',5)
set(gca, 'LineWidth', 3)
set(gca, 'TickLabelInterpreter', 'latex')
set(gca, 'FontSize', 50)
set(gcf, 'Color', 'white')
ylim([0.0, 0.1])

xline(17, 'k--', 'LineWidth', 2)
xline(53, 'k--', 'LineWidth', 2)
text(1, 0.09, {'Adding', 'obstacle 1'}, 'FontSize', 30)
text(23, 0.09, "Adding obstacle 2", 'FontSize', 30)
text(66, 0.09, "Replay trajectory", 'FontSize', 30)

%% plot kin_error
load('optimizer_stats_good.mat', 'time')
kin_err_wheel_1_z_end = kin_err_wheel_1_z_end(find(time > 125 & time < 172));
kin_err_wheel_2_z_end = kin_err_wheel_2_z_end(find(time > 125 & time < 172));
kin_err_wheel_3_z_end = kin_err_wheel_3_z_end(find(time > 125 & time < 172));
kin_err_wheel_4_z_end = kin_err_wheel_4_z_end(find(time > 125 & time < 172));
kin_err_wheel_1_z_init = kin_err_wheel_1_z_init(find(time > 125 & time < 172));
kin_err_wheel_2_z_init = kin_err_wheel_2_z_init(find(time > 125 & time < 172));
kin_err_wheel_3_z_init = kin_err_wheel_3_z_init(find(time > 125 & time < 172));
kin_err_wheel_4_z_init = kin_err_wheel_4_z_init(find(time > 125 & time < 172));

time = time(find(time > 125 & time < 172));
time = time - ones(1,length(time))*72;

% kin_err_wheel_1_z_end = kin_err_wheel_1_z_end(find(time < 100));
% kin_err_wheel_2_z_end = kin_err_wheel_2_z_end(find(time < 100));
% kin_err_wheel_3_z_end = kin_err_wheel_3_z_end(find(time < 100));
% kin_err_wheel_4_z_end = kin_err_wheel_4_z_end(find(time < 100));
% time = time(find(time < 100));

figure(2)
hold on
subplot(4,1,1)
plot(time, kin_err_wheel_1_z_end)
subplot(4,1,2)
plot(time, kin_err_wheel_2_z_end)
subplot(4,1,3)
plot(time, kin_err_wheel_3_z_end)
subplot(4,1,4)
plot(time, kin_err_wheel_4_z_end)