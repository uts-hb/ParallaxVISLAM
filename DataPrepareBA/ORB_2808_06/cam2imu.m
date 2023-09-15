clear;close all; clc;
load GT_P0_PA.mat
load extrinsic_kitti_0504.mat

% cam2imu_t = zeros(1,3);
% cam2imu_R = normalize(cam2imu_R);

GT_input_q = [];
GT_input_t = []; 
GT_output_q = [];
GT_output_t = []; 
for i = 1 : size(GT_P0,1)

    C_R = eul2rotm(GT_P0(i,1:3)); 
    C_t = GT_P0(i,4:6)'; 

    GT_input_q = [GT_input_q;rotm2quat(C_R)];
    GT_input_t = [GT_input_t;C_t'];

    I_R = C_R*cam2imu_R';
    I_t = C_t-I_R*cam2imu_t';

    C_T = [C_R,C_t; 0 0 0 1];
%     I_T = C_T*(CAM_2_IMU);
        GT_P0(i,1:3) = rotm2eul(I_R);
        GT_P0(i,4:6) = I_t';

    GT_output_q = [GT_output_q;rotm2quat(I_R)];
    GT_output_t = [GT_output_t;I_t'];

%     plotTransforms(GT_input_t,GT_input_q);
%     hold on
%     plotTransforms(GT_output_t,GT_output_q);

%     plotTransforms(GT_input_t,GT_input_q);

end 
% plot3(GT_input_t(:,1),GT_input_t(:,2),GT_input_t(:,3),'r');
% hold on;
% axis equal;
% plot3(GT_output_t(:,1),GT_output_t(:,2),GT_output_t(:,3),'--g');

save GT_P0_PA.mat GT_P0 

input_t = [];
input_q =[];
output_t =[]; 
output_q = [];
figure
for i = 1: size(GT_P0,1)
    file=strcat('Image',int2str(i),'.mat');
    load(file);    

    C_R = eul2rotm(Image(1,1:3)); 
%     C_R = normalize(C_R);
    C_t = Image(1,4:6)'; 

    input_q = [input_q;rotm2quat(C_R)];
    input_t = [input_t;Image(1,4:6)];

%     C_T = [C_R,C_t; 0 0 0 1];
%     I_T = (C_T*inv(CAM_2_IMU)); 
    I_R = C_R*cam2imu_R';
%     I_R = normalize(I_R);
    I_t = C_t-I_R*cam2imu_t';
    Image(1,1:3) = rotm2eul((I_R));
    Image(1,4:6) = I_t';

    output_q = [output_q;rotm2quat(I_R)];
    output_t = [output_t;I_t'];
    
%     plotTransforms(input_t,input_q);
%     hold on
%     plotTransforms(output_t,output_q);

    save(file, 'Image');
end 

%     plotTransforms(input_t,input_q);
%     hold on
%     plotTransforms(output_t,output_q,'FrameSize',5);
%     plot3(output_t(:,1),output_t(:,2),output_t(:,3))

%         plot3(input_t(:,1),input_t(:,2),input_t(:,3))

%     AXIS EQUAL;


change_t= [];
change_q = [];
for i = 1 : size(output_t,1)
    o_R = quat2rotm(output_q(i,:));
    o_t = output_t(i,:);

    oc_R = o_R*cam2imu_R; 
    oc_t = o_t' + o_R*cam2imu_t';

    change_t= [change_t;oc_t'];
    change_q = [change_q; rotm2quat(oc_R)];

end 

    plot3(change_t(:,1),change_t(:,2),change_t(:,3));
    axis equal;
