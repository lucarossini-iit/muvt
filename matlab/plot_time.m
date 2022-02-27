clear all
close all
clc

load('locomotion.mat')
init_time = 20;
fin_time = 90;
replay_time = 35;

%% plot opt time
figure(1)
hold on 
grid on

% cut vertices
opt_time = opt_time(find(time > init_time & time < fin_time));
time = time(find(time > init_time & time < fin_time));

time = time - ones(1,length(time))*time(1);

% filter
d1 = designfilt('lowpassiir','FilterOrder',12, ...
    'HalfPowerFrequency',0.15,'DesignMethod','butter');
opt_time_filt = filtfilt(d1, opt_time);

% plot
plot(time, opt_time, 'LineWidth', 1, 'Color', '#9FBBD6')
plot(time, opt_time_filt, 'LineWidth',5)
set(gca, 'LineWidth', 3)
% set(gca, 'TickLabelInterpreter', 'latex')
set(gca, 'FontSize', 30)
set(gcf, 'Color', 'white')
ylim([0.0, 0.1])
xlabel('Time [s]')
ylabel('Planning Time [s]')

xline(replay_time, 'k--', 'LineWidth', 2)
text(10, 0.095, "Setup", 'FontSize', 40)
text(40, 0.095, "Replay trajectory", 'FontSize', 40)

%% plot kin_error
load('locomotion.mat', 'time');
kin_err_wheel_1_z_end = kin_err_wheel_1_z_end(find(time > init_time + replay_time & time < fin_time));
kin_err_wheel_2_z_end = kin_err_wheel_2_z_end(find(time > init_time + replay_time & time < fin_time));
kin_err_wheel_3_z_end = kin_err_wheel_3_z_end(find(time > init_time + replay_time & time < fin_time));
kin_err_wheel_4_z_end = kin_err_wheel_4_z_end(find(time > init_time + replay_time & time < fin_time));
kin_err_wheel_1_z_init = kin_err_wheel_1_z_init(find(time > init_time + replay_time & time < fin_time));
kin_err_wheel_2_z_init = kin_err_wheel_2_z_init(find(time > init_time + replay_time & time < fin_time));
kin_err_wheel_3_z_init = kin_err_wheel_3_z_init(find(time > init_time + replay_time & time < fin_time));
kin_err_wheel_4_z_init = kin_err_wheel_4_z_init(find(time > init_time + replay_time & time < fin_time));

% kin_err_wheel_1_z_init = kin_err_wheel_1_z_init ./ (ones(1, length(kin_err_wheel_1_z_init))*2);
% kin_err_wheel_2_z_init = kin_err_wheel_2_z_init ./ (ones(1, length(kin_err_wheel_2_z_init))*2);
% kin_err_wheel_3_z_init = kin_err_wheel_3_z_init ./ (ones(1, length(kin_err_wheel_3_z_init))*2);
% kin_err_wheel_4_z_init = kin_err_wheel_4_z_init ./ (ones(1, length(kin_err_wheel_4_z_init))*2);

time = time(find(time > init_time + replay_time & time < fin_time));
time = time - ones(1,length(time))*init_time;

% kin_err_wheel_1_z_end = kin_err_wheel_1_z_end(find(time < 100));
% kin_err_wheel_2_z_end = kin_err_wheel_2_z_end(find(time < 100));
% kin_err_wheel_3_z_end = kin_err_wheel_3_z_end(find(time < 100));
% kin_err_wheel_4_z_end = kin_err_wheel_4_z_end(find(time < 100));
% time = time(find(time < 100));

fig = figure(2)

subplot(4,1,1)
hold on
grid on
plot(time, kin_err_wheel_1_z_init, 'LineWidth', 5)
plot(time, kin_err_wheel_1_z_end, 'LineWidth', 5)
set(gca, 'LineWidth', 2)
% set(gca, 'TickLabelInterpreter', 'latex')
set(gca, 'FontSize', 30)
set(gcf, 'Color', 'white')
legend('first vertex','last vertex')
ylim([-0.15, 0.15])
% title('Wheel FL')
subplot(4,1,2)
hold on
grid on
plot(time, kin_err_wheel_2_z_init, 'LineWidth', 5)
plot(time, kin_err_wheel_2_z_end, 'LineWidth', 5)
set(gca, 'LineWidth', 2)
% set(gca, 'TickLabelInterpreter', 'latex')
set(gca, 'FontSize', 30)
set(gcf, 'Color', 'white')
ylim([-0.15, 0.15])
% title('Wheel FR')
subplot(4,1,3)
hold on
grid on
plot(time, kin_err_wheel_3_z_init, 'LineWidth', 5)
plot(time, kin_err_wheel_3_z_end, 'LineWidth', 5)
set(gca, 'LineWidth', 2)
% set(gca, 'TickLabelInterpreter', 'latex')
set(gca, 'FontSize', 30)
set(gcf, 'Color', 'white')
ylim([-0.15, 0.15])
% title('Wheel RL')
subplot(4,1,4)
hold on
grid on
plot(time, kin_err_wheel_4_z_init, 'LineWidth', 5)
plot(time, kin_err_wheel_4_z_end, 'LineWidth', 5)
set(gca, 'LineWidth', 2)
% set(gca, 'TickLabelInterpreter', 'latex')
set(gca, 'FontSize', 30)
set(gcf, 'Color', 'white')
ylim([-0.15, 0.15])
% title('Wheel RR', 'FontSize', 20)

han=axes(fig,'visible','off'); 
han.XLabel.Visible='on';
han.YLabel.Visible='on';
xlabel(han, 'Time [s]', 'FontSize', 30);
label = ylabel(han, 'Error [m]', 'FontSize', 30);
label.Position(1) = -0.09;

%% collision error
load('locomotion.mat', 'time');
coll_err_init = coll_err_init(find(time > init_time + replay_time & time < fin_time));
coll_err_end = coll_err_end(find(time > init_time + replay_time & time < fin_time));
time = time(find(time > init_time + replay_time & time < fin_time));
time = time - ones(1, length(time)) * init_time;

figure(3)
hold on
grid on
set(gca, 'LineWidth', 2)
% set(gca, 'TickLabelInterpreter', 'latex')
set(gca, 'FontSize', 30)
set(gcf, 'Color', 'white')
plot(time, coll_err_init, 'LineWidth', 5)
plot(time, coll_err_end, 'LineWidth', 5)
xlabel('Time [s]')
ylabel('Error')
legend('first vertex', 'last vertex')


%% manipulation
load('manipulation_human.mat')
init_time = 102;
fin_time = 102+42;

fig4 = figure(4)

% cut vertices
opt_time = opt_time(find(time > init_time & time < fin_time));
coll_err_end = coll_err_end(find(time > init_time & time < fin_time));
coll_err_init = coll_err_init(find(time > init_time & time < fin_time));
time = time(find(time > init_time & time < fin_time));

time = time - ones(1,length(time))*init_time;

% filter
d1 = designfilt('lowpassiir','FilterOrder',12, ...
    'HalfPowerFrequency',0.15,'DesignMethod','butter');
opt_time_filt = filtfilt(d1, opt_time);

% plot
subplot(2,1,1)
hold on
grid on
plot(time, opt_time, 'LineWidth', 1, 'Color', '#9FBBD6')
plot(time, opt_time_filt, 'LineWidth',5)
set(gca, 'LineWidth', 3)
% set(gca, 'TickLabelInterpreter', 'latex')
set(gca, 'FontSize', 30)
set(gcf, 'Color', 'white')
ylim([0.0, 0.04])
xlim([0.0, 42])
ylabel('Planning Time [s]')

subplot(2,1,2)
hold on
grid on
plot(time, coll_err_init, 'LineWidth', 5)
plot(time, coll_err_end, 'LineWidth', 5)
set(gca, 'LineWidth', 3)
% set(gca, 'TickLabelInterpreter', 'latex')
set(gca, 'FontSize', 30)
set(gcf, 'Color', 'white')
xlim([0.0, 42])
ylabel('Error')
legend('first vertex', 'last vertex')
han=axes(fig4,'visible','off'); 
han.XLabel.Visible='on';
han.YLabel.Visible='on';
label = xlabel(han, 'Time [s]', 'FontSize', 30);
label.Position(2) = -0.07;



