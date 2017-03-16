
plot(actualTravel2.time, actualTravel2.signals.values);
matlab2tikz('travelunderUnderModelbasis.tex','parseStrings',true, 'height', '\figureheight', 'width', '\figurewidth');