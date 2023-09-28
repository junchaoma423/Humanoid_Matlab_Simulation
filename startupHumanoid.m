% Humanoid Robot Startup Script

%% Clear everything
clc;
clear;
% clf;

%% Add folders to the path
addpath(genpath('Functions'), ...               % 
        genpath('STL files'), ...
        genpath('Spatialv2'));                  % 

%% Load basic Humanoid robot parameters
Humanoid_Parameters
