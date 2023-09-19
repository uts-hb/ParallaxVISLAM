 function feat_pos = FuncInputBA_ORB(PVector,Feature)

 load extrinsic_kitti_0504.mat

% Data.xyzpoints
% Pose = reshape(PVector.Pose,6,[])';
feat_pos = [];
tm_temp =[]; 
num_feat = size(Feature, 1); 


for i = 1 : size(PVector.Pos,2)
    R_imu = eul2rotm(PVector.Rot{1,i});
    t_imu = PVector.Pos(:,i);
    R_cam = R_imu*cam2imu_R;
    PVector.Pos(:,i) = t_imu + R_imu*cam2imu_t';
    PVector.Rot{1,i} = rotm2eul(R_cam);
end 



for i = 1:num_feat
    Xj = FuncXj(PVector.Feature(3*(i-1)+1),PVector.Feature(3*(i-1)+2));
    R = RMatrixYPR22_BA(PVector.Rot{1,Feature(i,3)}(1),PVector.Rot{1,Feature(i,3)}(2),PVector.Rot{1,Feature(i,3)}(3));
%     R = RMatrixYPR22_BA(PVector.Pose(6*(Feature(i,3))-5),PVector.Pose(6*(Feature(i,3))-4),PVector.Pose(6*(Feature(i,3))-3));
    tm = PVector.Pos(:,Feature(i,3))';
%     tm = PVector.Pose(6*((Feature(i,3)-1))+4:6*((Feature(i,3)-1))+6)';
    tm_temp = [tm_temp; tm];
    
    if Feature(i,4) ~= 0
        ta = PVector.Pos(:,Feature(i,4))';
%         ta = PVector.Pose(6*((Feature(i,4)-1))+4:6*((Feature(i,4)-1))+6)';
    else
        ta = tm;
    end
    
    phi = acos(dot(Xj, ((ta-tm)/norm(ta-tm))'));
    
    if Feature(i,4) ~= 0
        depth = (sin(PVector.Feature(3*(i-1)+3)+phi)*norm(ta-tm))/sin(PVector.Feature(3*(i-1)+3));
    else
        depth = 0;
    end
    
    feat_pos(:,end+1)= depth*Xj + tm';
end

