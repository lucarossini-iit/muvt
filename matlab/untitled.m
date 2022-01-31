clear all 
close all
clc

% draw objective functions plots
% 1 - EdgeCollision
x = linspace(0, 0.2, 5000);
eps = 0.05;
r = 0.05;
S = 0.025;
n = 2;
y = [];
for ii = 1:length(x)
    if x(ii) < r + eps
        y(ii) = ((-x(ii) - (-r - eps))/S)^n;
    else
        y(ii) = 0;
    end
end

figure(1)
plot(x,y, 'LineWidth', 6)
grid on
xline(0.1, 'k--', 'LineWidth', 6)
set(gca, 'LineWidth', 3)
set(gca, 'TickLabelInterpreter', 'latex')
set(gca, 'FontSize', 65)
set(gca, 'XTick', 0.1, 'XTickLabel', '$r + \varepsilon$')
set(gcf, 'Color', 'white')
axis equal
xlim([-0.01, 0.2])
ylim([-0.01, 0.2])

% 2 - EdgeRobotVel
x = linspace(-1.1, 1.1, 1000);
S = 5;
y = [];
for ii = 1:length(x)
    y(ii) = 1/exp(x(ii) + 1)^S + exp(x(ii) - 1)^S;
end
figure(2)
plot(x,y, 'LineWidth', 6)
grid on
xline(-1, 'k--', 'LineWidth', 6)
xline(1, 'k--', 'LineWidth', 6)
set(gca, 'LineWidth', 3)
set(gca, 'TickLabelInterpreter', 'latex')
set(gca, 'FontSize', 65)
set(gca, 'XTick', [-1.0, 0.0,  1.0], 'XTickLabel', {'$min$' , 0.0,  '$max$'})
set(gcf, 'Color', 'white')
axis equal
xlim([-1.1, 1.1])
ylim([-0.1, 2.1])