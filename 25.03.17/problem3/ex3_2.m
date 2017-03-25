%% Plotting
%matlab2tikz('tenQ.tex','parseStrings',true, 'height', '\figureheight', 'width', '\figurewidth');
clc;
%close all;

figure(1)
grid
plot(t', x1*180/pi,actualTravel2.time, actualTravel2.signals.values);
legend('\lambda_{ref}', '\lambda');
ylabel('\lambda [deg]');
xlabel('Time [s]');


figure(2)
grid
plot(t', u*180/pi,actual_p_ref.time, actual_p_ref.signals.values);
legend('u_{ref}', 'u');
ylabel('Pitch [deg]');
xlabel('Time [s]');

%plot(actual_p_ref.time, actual_p_ref.signals.values);

