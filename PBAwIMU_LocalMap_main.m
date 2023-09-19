%% PBA_IMU_SO3.m
% Monocular version of Parallax Bundle Adjustment in IMU frame. 
% It can be optimized with pre-integrated IMU data. 

close all;clc;clear;
addpath('utils');

%% Load extrinsic and intrinsic matrix of KITTI dataset 
load extrinsic_kitti.mat;
K = textread('calSBA_kitti.txt');

%% Load preintegrated-IMU data 
% Dataset = 'KITTI_06'; % 1~412
Dataset = 'KITTI_07';  % 1~412
% Dataset = 'KITTI_09'; % 1~677

%% Number of Images 
start_ImageNum = 1; 
end_ImageNum = 412; 

%%
switch Dataset;
    case 'KITTI_06';
        load imu_pre_06.mat 
        load trajectory_06.mat
    case 'KITTI_07';
        load imu_pre_07.mat
        load trajectory_07.mat
    case 'KITTI_09';
        load imu_pre_09.mat
        load trajectory_09.mat
end 

%% Load Ground-truth pose of the dataset
GTName = strcat('DataPrepareBA/',Dataset,'/GT_P0_PA.mat');
load(GTName);

%% Plot GT-IMU pose
figure(2); 
plot3(GT_P0(start_ImageNum:end_ImageNum,4),GT_P0(start_ImageNum:end_ImageNum,5),GT_P0(start_ImageNum:end_ImageNum,6),'-r');
axis equal; grid on; hold on; 

%% Initializing for PBA
Feature = zeros(10000,180);
xVector.u = []; xVector.PID = []; xVector.FID = [];
PVector.Pos = []; PVector.Rot = {};  PVector.Feature = []; PVector.ID = []; PVector.Info = sparse([]);
PVector.v = []; PVector.bg = []; PVector.ba = [];
GT_T_start = [eul2rotm(GT_P0(start_ImageNum,1:3)),GT_P0(start_ImageNum,4:6)'; 0 0 0 1]; 
file = strcat('DataPrepareBA/',Dataset,'/Image',int2str(start_ImageNum));
load(file);
Image_T_start = [eul2rotm(Image(1,1:3)),Image(1,4:6)'; 0 0 0 1]; 
for i=start_ImageNum:end_ImageNum;
    file = strcat('DataPrepareBA/',Dataset,'/Image',int2str(i));
	load(file);
	fprintf('%s\n', file);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Transforming the start point using the GT 
    Image_T = [eul2rotm(Image(1,1:3)),Image(1,4:6)'; 0 0 0 1]; 
    Image_T = GT_T_start*inv(Image_T_start)*Image_T;
    Image(1,1:3) = rotm2eul(Image_T(1:3,1:3));
    Image(1,4:6) = Image_T(1:3,4)';
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	xVector = FuncGetxVector(xVector,Image,i,start_ImageNum);
	[PVector,Feature] = FuncGetInitial3_02(PVector,Feature,Image,i,K,CAM_2_IMU,start_ImageNum);
    PVector.v = [PVector.v, EST_pose{1,i}.v];
    PVector.bg = [PVector.bg, zeros(3,1)];
    PVector.ba = [PVector.ba, zeros(3,1)];
end;
PVector.v(:,1) = gt_vel(:,start_ImageNum);

%% Delete the feature only observed once
[Feature,PVector,xVector,feature_id_change] = delete_feature_one(Feature,PVector,xVector);

%% Plot initial guess in IMU frame
pos_initial = PVector.Pos; 
plot3(pos_initial(1,:),pos_initial(2,:),pos_initial(3,:),'--g');
hold on; 

%% Pre-integrated IMU for new Local Map
pre = {};
for i = start_ImageNum : end_ImageNum-1
    pre{end+1} = pre_imu{1,i};
end

file = strcat(Dataset,'_',int2str(start_ImageNum),'_',int2str(end_ImageNum));
diary(file);

%% Least Squares GN
tic
[PVector,Reason,Info,objFun,errorPBA,errorIMU] = FuncLeastSquares_w_IMU(xVector,PVector,Feature,K,CAM_2_IMU,pre,Dataset);
toc
diary off;
%% Levenberg-Marquardt Iteration SBA
%     [PVector,Reason,Info,objFun,errorPBA,errorIMU] = FuncLeastSquaresLMSBA_w_imu(xVector,PVector,Feature,K,CAM_2_IMU,pre,Dataset);
% diary off;

%% Changing from IMU to CAM frame to outfile txt for using evo_traj
% PVector_cam_optimal = pose_imu2cam(PVector);
% pos_cam_optimal = PVector_cam_optimal.Pos;
% PBA_so3_traj = [];
% switch Dataset;
%     case 'KITTI_06';
%         load time_image_06.mat;
%     case 'KITTI_07';
%         load time_image_07.mat;
%     case 'KITTI_09';
%         load time_image_09.mat;
% end
% rpg_PBA_so3_traj = [];
% for i = 1 : size(PVector_cam_optimal.Pos,2)
%     vector=[];
%     q = rotm2quat(PVector_cam_optimal.Rot{1,i});
%     q = [q(2:4),q(1)];
%     for j=1:3
%         vector=[vector, PVector_cam_optimal.Rot{1, i}(j,:), PVector_cam_optimal.Pos(j,i)];
%     end
%     PBA_so3_traj = [PBA_so3_traj;vector];
%     rpg_PBA_so3_traj = [rpg_PBA_so3_traj;time_image(i,1), PVector_cam_optimal.Pos(:,i)',q];
% end
% dlmwrite('PBA_IMU_traj_2808_local_06.txt',PBA_so3_traj,'delimiter',' ')
% dlmwrite('rpg_PBA_IMU_traj_2808_local_06.txt',rpg_PBA_so3_traj,'delimiter',' ')


%% Plot trajectory in IMU frame
pos_optimal = PVector.Pos;
plot3(pos_optimal(1,:),pos_optimal(2,:),pos_optimal(3,:),'b--');
legend('GT', 'Initial' ,'OPT');

fprintf('Reason is %d\n', Reason);
PVector.ID = Feature(:,1:4);
PVector.Info = Info;

save(file);

