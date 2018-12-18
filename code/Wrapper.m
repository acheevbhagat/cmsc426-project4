%% Wrapper for the CMSC426Course final project at University of Maryland, College Park
% Code by: Nitin J. Sanket (nitinsan@terpmail.umd.edu)
%          (Originally for CMSC828T)

clc
clear all
close all

%% Add ToolBox to Path eg. ToolboxPath = 'gtsam_toolbox';
%ToolboxPath = '~/Documents/cmsc426/gtsam_toolbox'
%addpath(ToolboxPath);

%% Load Data
% Download data from the following link: 
% https://drive.google.com/open?id=1ZFXZEv4yWgaVDE1JD6-oYL2KQDypnEUU
load('../DataMappingNew.mat');
load('../CalibParams.mat');

%% SLAM Using GTSAM
[LandMarksComputed, AllPosesComputed] = SLAMusingGTSAM(DetAll, K, TagSize, qIMUToC, TIMUToC,...
                                                IMU);
