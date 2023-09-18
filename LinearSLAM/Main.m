clc;
clear;

%% Map Joining Method

% Method = 'Sequential';
Method = 'Divide&Conquer';

%% Dataset Type

%  DataType = '2D Pose Feature';
DataType = '3D Pose Feature';
% DataType = '2D Pose Graph';
% DataType = '3D Pose Graph';

    %% 3D Pose Feature Datasets 

    %% KITTT Dataset Sequence 06, 07, 08 with differenent number of Local Map  
Dataset = '06_2';
% Dataset = '06_4';
% Dataset = '06_6';
% Dataset = '06_8';
% Dataset = '06_10';
% Dataset = '07_2';
% Dataset = '07_4';
% Dataset = '07_6';
% Dataset = '07_8';
% Dataset = '07_10';
% Dataset = '09_2';
% Dataset = '09_4';
% Dataset = '09_6';
% Dataset = '09_8';
%% Direction and Local Map Number
switch Dataset;
    case '06_2';
        LocalMapNum = 2;
        Direction = '06_2/';
    case '06_4';
        LocalMapNum = 4;
        Direction = '06_4/';
    case '06_6';
        LocalMapNum = 6;
        Direction = '06_6/';
    case '06_8';
        LocalMapNum = 8;
        Direction = '06_8/';
    case '06_10';
        LocalMapNum = 10;
        Direction = '06_10/';
    case '07_2';
        LocalMapNum = 2;`
        Direction = '07_2/';
    case '07_4';
        LocalMapNum = 4;
        Direction = '07_4/';
    case '07_6';
        LocalMapNum = 6;
        Direction = '07_6/';
    case '07_8';
        LocalMapNum = 8;
        Direction = '07_8/';
    case '07_10';
        LocalMapNum = 10;
        Direction = '07_10/';
    case '09_2';
        LocalMapNum = 2;
        Direction = '09_2/';
    case '09_4';
        LocalMapNum = 4;
        Direction = '09_4/';
    case '09_6';
        LocalMapNum = 6;
        Direction = '09_6/';
    case '09_8';
        LocalMapNum = 8;
        Direction = '09_8/';        
end;

%% Load Files
for i=1:LocalMapNum;
    file=strcat(Direction,'localmap_',int2str(i)); 
	LM{i} = load(file);
end;

%% Linear Map Joining
diary time; 
tic;

switch Method;
    case 'Sequential';
        switch DataType;
            case '2D Pose Feature';
                [GMap] = LMJ_PF2D_Sequential(LM,LocalMapNum); % 2D Pose Feature Sequential
            case '3D Pose Feature';
                [GMap] = LMJ_PF3D_Sequential(LM,LocalMapNum); % 3D Pose Feature Sequential
            case '2D Pose Graph';
                [GMap] = LMJ_PG2D_Sequential(LM,LocalMapNum); % 2D Pose Graph Sequential
            case '3D Pose Graph';
                [GMap] = LMJ_PG3D_Sequential(LM,LocalMapNum); % 3D Pose Graph Sequential
        end;
    case 'Divide&Conquer';
        switch DataType;
            case '2D Pose Feature';
                [GMap,Level_Map] = LMJ_PF2D_Divide_Conquer(LM,LocalMapNum); % 2D Pose Feature Divide and Conquer
            case '3D Pose Feature';
                [GMap,Level_Map] = LMJ_PF3D_Divide_Conquer(LM,LocalMapNum); % 3D Pose Feature Divide and Conquer
            case '2D Pose Graph';
                [GMap,Level_Map] = LMJ_PG2D_Divide_Conquer(LM,LocalMapNum); % 2D Pose Graph Divide and Conquer
            case '3D Pose Graph';
                [GMap,Level_Map] = LMJ_PG3D_Divide_Conquer(LM,LocalMapNum); % 3D Pose Graph Divide and Conquer
        end;
end

toc
diary off;
%% Plot Result Figure
switch DataType;
    case '2D Pose Feature';
        [Pose,Feature] = FuncPlotFigure_PF2D(GMap);
    case '3D Pose Feature';
        [Pose,Feature] = FuncPlotFigure_PF3D(GMap);
    case '2D Pose Graph';
        [Pose] = FuncPlotFigure_PG2D(GMap);
    case '3D Pose Graph';
        [Pose] = FuncPlotFigure_PG3D(GMap);
end;

load extrinsic_kitti_0504.mat
temp_R = eye(3,3)*cam2imu_R;
temp_t = zeros(3,1) + eye(3,3)*cam2imu_t';
Pose = [0 0 0 0 0 0 0; Pose];
LinearSLAM_traj = [];
load time_image_2808_09.mat
rpg_LinearSLAM_traj = [];
for i = 1 : size(Pose,1)
    vector=[];
    R_ = eul2rotm(Pose(i,5:7))*cam2imu_R;
    q = rotm2quat(R_);
    q = [q(2:4),q(1)];
    t_ = Pose(i,2:4)' + eul2rotm(Pose(i,5:7))*cam2imu_t';
    for j=1:3
        vector=[vector, R_(j,:), t_(j,1)];
    end
    LinearSLAM_traj = [LinearSLAM_traj;vector];
    rpg_LinearSLAM_traj = [rpg_LinearSLAM_traj;time_image(i,1),t_' ,q];
end

dlmwrite('LinearSLAM_traj_2808_09.txt',LinearSLAM_traj,'delimiter',' ')
dlmwrite('rpg_LinearSLAM_traj_09.txt',rpg_LinearSLAM_traj,'delimiter',' ')

