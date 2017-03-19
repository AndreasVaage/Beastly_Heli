%% Plotting
%matlab2tikz('tenQ.tex','parseStrings',true, 'height', '\figureheight', 'width', '\figurewidth');
clc;
%close all;

figure(1)

subplot(211)
grid
plot(t', x1*180/pi,actualTravel2.time, actualTravel2.signals.values);
legend('e_{ref}', 'e');
ylabel('e [deg]');
subplot(212)
plot(t', x5*180/pi,actualTravel2.time, actualElevation.signals.values);
legend('\lambda_{ref}', '\lambda');
ylabel('\lambda [deg]');
xlabel('Time [s]');


figure(2)
subplot(211)
grid
plot(t', u1*180/pi,actual_p_ref.time, actual_p_ref.signals.values);
legend('u_{ref}', 'u');
ylabel('Pitch [deg]');
subplot(212)
plot(t', u2*180/pi,actual_e_ref.time, actual_e_ref.signals.values);
legend('u_{ref}', 'u');
ylabel('e [deg]');
xlabel('Time [s]');
hold off;
