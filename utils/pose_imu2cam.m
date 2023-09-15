function PVector_cam = pose_imu2cam(PVector)

load extrinsic_kitti_0504.mat

PVector_cam = PVector; 

for i = 1 : size(PVector.Pos,2)

    R = PVector.Rot{1,i}; 
    t = PVector.Pos(:,i); 

    PVector_cam.Rot{1,i} = R*cam2imu_R; 
    PVector_cam.Pos(:,i) = t + R*cam2imu_t';



end 