function [upper,lower] = makeAxisLimits(maxValue,minValue,slack)
% Function that calculates upper and lower limits of a graph with slack of
% 'slack', given data max and min values
% 'slack' is given as a percentage, e.g. 0.1 = 10%

if minValue<0
    lower = (1+slack)*minValue;
else
    lower = (1-slack)*minValue;
end

if maxValue>0
    upper = (1+slack)*maxValue;
else
    upper = (1-slack)*maxValue;
end


