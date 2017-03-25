%% Plotting
%matlab2tikz('tenQ.tex','parseStrings',true, 'height', '\figureheight', 'width', '\figurewidth');
clc;
%close all;

figure
hold on;
subplot(211)
grid
plot(t', x1*180/pi,actualTravel2.time, actualTravel2.signals.values);
legend('\lambda_{ref}', '\lambda');
ylabel('\lambda [deg]');
subplot(212)
plot(t', x5*180/pi,actualTravel2.time, actualElevation.signals.values);

legend('e_{ref}', 'e');
ylabel('e [deg]');
xlabel('Time [s]');
hold off;
