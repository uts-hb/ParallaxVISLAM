clear all; close all; clc; 

load GT_P0_PA.mat
% load imu_pre_2808_07.mat
pose = [];
load("Image100.mat");
pre_scale = norm(Image(1,4:6));
load trajectory_2808_07.mat
new_scale = norm(EST(100,4:6));

orig_pose = [];
corrected_pose =[]; 


T_s = [(new_scale/pre_scale)*eye(3), zeros(3,1); 0 0 0 1]; 

for i = 1: size(GT_P0,1)
    file=strcat('Image',int2str(i),'.mat');
    load(file);   
    orig_pose = [orig_pose; Image(1,4:6)]; 
    Temp = T_s*[Image(1,4:6)';1];
    Image(1,4:6) = Temp(1:3,1)';
    corrected_pose =[corrected_pose;Image(1,4:6)]; 
    save(file, 'Image');
end 

plot3(orig_pose(:,1),orig_pose(:,2),orig_pose(:,3));
hold on; axis equal; 
plot3(corrected_pose(:,1),corrected_pose(:,2),corrected_pose(:,3));
plot3(GT_P0(:,4),GT_P0(:,5),GT_P0(:,6));
