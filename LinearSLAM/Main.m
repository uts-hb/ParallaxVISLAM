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

Dataset = '09_8';

%% Direction and Local Map Number
switch Dataset;
    case 'KITTI_09';
        LocalMapNum = 2;
        Direction = 'KITTI_09/';
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
    case '09_10';
        LocalMapNum = 10;
        Direction = '09_10/';
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

% figure(2); 
% % plot3(Pose(:,2),Pose(:,3),Pose(:,4),'r--');
% plot(Pose(:,2),Pose(:,3),'r-');
% axis equal; grid on;hold on;
% load compare_1_438.mat
% plot(t_local1(1,:),t_local1(2,:),'g--');
% plot(t_local2_to_1(1,:),t_local2_to_1(2,:),'k--');
% plot(t_gt(1,:),t_gt(2,:), 'b--');
% load local_map_1_438.mat
% T_1_438_1 = [PVector.Rot{1,1},PVector.Pos(:,1);0 0 0 1];
% t_1_438 = [];
% for i = 1 : size(PVector.Rot,2)
%     temp_T = inv(T_1_438_1)*[PVector.Rot{1,i},PVector.Pos(:,i);0 0 0 1];    
%     t_1_438 = [t_1_438,temp_T(1:3,4)];
% end 
% plot(t_1_438(1,:),t_1_438(2,:),'c--');
% legend('LinearSLAM', 'LocalMap1', 'LocalMap2', 'GT', 'FullBatch');

% legend('LinearSLAM', 'LocalMap1', 'LocalMap2', 'GT');
% legend('LinearSLAM', 'LocalMap1', 'LocalMap2');

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

