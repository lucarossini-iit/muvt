clear all 
close all
clc

% draw objective functions plots
% 1 - EdgeCollision
x = linspace(-1.0, 2.0, 5000);
r = 0.05;
S = 0.05;
n = 2;
y = [];
for ii = 1:length(x)
    if x(ii) < r + eps
        y(ii) = ((-x(ii) - (-r))/S)^n;
    else
        y(ii) = 0;
    end
end

figure(1)
plot(x,y, 'LineWidth', 6)
grid on
hold on
xline(0.05, 'k--', 'LineWidth', 6)
set(gca, 'LineWidth', 3)
set(gca, 'TickLabelInterpreter', 'latex')
set(gca, 'FontSize', 65)
set(gca, 'XTick', 0.1, 'XTickLabel', '$d_{\mathrm{th}}$')
set(gcf, 'Color', 'white')
axis equal
xlim([-1.0, 2.0])
ylim([-0.01, 3.0])

x1 = linspace(-1.0, 2.0, 5000);
r = 0.05;
S = 0.5;
n = 2;
y1 = [];
for ii = 1:length(x)
    if x1(ii) < r
        y1(ii) = ((-x1(ii) - (-r))/S)^n;
    else
        y1(ii) = 0;
    end
end

plot(x1,y1,'LineWidth',6)
legend('$S = 0.05$', '', '$S = 0.5$', 'interpreter', 'latex')

% 2 - EdgeRobotVel
x = linspace(-1.5, 1.5, 1000);
S = 10;
y = [];
for ii = 1:length(x)
    y(ii) = 1/exp(x(ii) + 1)^S + exp(x(ii) - 1)^S;
end
figure(2)
plot(x,y, 'LineWidth', 6)
grid on
hold on
xline(-1, 'k--', 'LineWidth', 6)
xline(1, 'k--', 'LineWidth', 6)
set(gca, 'LineWidth', 3)
set(gca, 'TickLabelInterpreter', 'latex')
set(gca, 'FontSize', 65)
set(gca, 'XTick', [-1.0, 0.0,  1.0], 'XTickLabel', {'$min$' , 0.0,  '$max$'})
set(gcf, 'Color', 'white')
axis equal
xlim([-1.5, 1.5])
ylim([-0.1, 2.9])

x = linspace(-1.5, 1.5, 1000);
S = 2;
y = [];
for ii = 1:length(x)
    y(ii) = 1/exp(x(ii) + 1)^S + exp(x(ii) - 1)^S;
end

plot(x,y,'LineWidth',6)
legend('$S = 10$', '', '', '$S = 2$', 'interpreter', 'latex')