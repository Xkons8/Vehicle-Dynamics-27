function [outputArg1,outputArg2] = brake_split(inputArg1,inputArg2)
%BRAKE_SPLIT Determines the brake bias split according to a better model
%where every bias is not ideal
%   Detailed explanation goes here
arguments (Input)
    inputArg1
    inputArg2
end

arguments (Output)
    outputArg1
    outputArg2
end

outputArg1 = inputArg1;
outputArg2 = inputArg2;
end