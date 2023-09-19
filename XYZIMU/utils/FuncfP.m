
function [uvcomp] = FuncfP(PVector,Feature,K,extrinsic)

[nRowNumF,] = size(Feature);
uvcomp = zeros(1);
k = 0;

for j=1:nRowNumF;
    Xj = Feature(j,1:3);
    for num=1:Feature(j,4);
        i = Feature(j,3*(num-1)+5)+1;

        R_imu = PVector.Rot{1,i};
        t_imu = PVector.Pos(:,i);
        R_cam = R_imu*extrinsic(1:3,1:3); 
        t_cam = t_imu + R_imu*extrinsic(1:3,4);
%         eul_cam = rotm2eul(R_cam);
%         Pi = [eul_cam, t_cam'];
        Pi.R = R_cam;
        Pi.p = t_cam';
        [u,v] = Funcfuv(Pi,Xj,K);
        k=k+1;
        uvcomp(k,1) = u;
        k=k+1;
        uvcomp(k,1) = v;
    end;
end;