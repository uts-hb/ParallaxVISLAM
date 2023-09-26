function [W] = funcWeight(xVector,PVector,pre_imu,Dataset)

W_ID1 = [];
W_ID2 = [];
VAL = [];

W_BA_ID1 = [1:size(xVector.u,1)];
W_BA_ID2 = [1:size(xVector.u,1)];
val_BA = [ones(1,size(xVector.u,1))];

val_IMU = [];
W_IMU_ID1 = [];
W_IMU_ID2 = [];


for i = 1 : size(PVector.Pos,2)-1
    W_IMU_ID1 = [W_IMU_ID1, repelem([size(xVector.u,1)+15*(i-1)+1:size(xVector.u,1)+15*(i-1)+9],9)];
    W_IMU_ID2 = [W_IMU_ID2,repmat([size(xVector.u,1)+15*(i-1)+1:size(xVector.u,1)+15*(i-1)+9],[1,9])];
    val_IMU = [val_IMU, reshape(inv(pre_imu{1,i}.cov),1,[])];
%     val_IMU = [val_IMU, reshape(eye(9,9),1,[])];

end

cov_b_ang = ((0.001)^2)*eye(3,3);
cov_b_acc = ((1.0e-4)^2)*eye(3,3);
% cov_b_ang = ((0.02)^2)*eye(3,3);
% cov_b_acc = ((4.0e-5)^2)*eye(3,3);
W_bg = inv(cov_b_ang);
W_ba = inv(cov_b_acc);
% W_bg = eye(3,3);
% W_ba = eye(3,3);


val_B = [];
W_B_ID1 = [];
W_B_ID2 = [];
for i = 1 : size(PVector.Pos,2)-1
    W_B_ID1 = [W_B_ID1, size(xVector.u,1)+15*(i-1)+10:size(xVector.u,1)+15*(i-1)+15];
    W_B_ID2 = [W_B_ID2, size(xVector.u,1)+15*(i-1)+10:size(xVector.u,1)+15*(i-1)+15];
    val_B = [val_B, diag(W_bg)',diag(W_ba)'];
end

W_ID1 = [W_BA_ID1, W_IMU_ID1, W_B_ID1];
W_ID2 = [W_BA_ID2, W_IMU_ID2, W_B_ID2];

switch Dataset;
    case 'KITTI_06';
        VAL = [val_BA, 10*val_IMU, 10*val_B]; % Best for dataset 06
    case 'KITTI_07';
        VAL = [val_BA, 150*val_IMU, 150*val_B]; % Best for dataset 07
    case 'KITTI_09';
        VAL = [val_BA, 60*val_IMU, 60*val_B]; % best for dataset 09 
end 

W = sparse(W_ID1,W_ID2,VAL);

