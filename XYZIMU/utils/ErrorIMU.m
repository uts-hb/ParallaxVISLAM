function [IMU_Error] = ErrorIMU(PVector, pre_imu)

IMU_Error = []; 
g = [0;0;-9.80655];

for i = 1 : size(PVector.Pos,2)-1
    err = zeros(15,1);
    t = sum(pre_imu{1, i}.dt);
    t22 = t*t/2;

    delta_bg = PVector.bg(:,i+1) - pre_imu{1,i}.bg;
    delta_ba = PVector.ba(:,i+1) - pre_imu{1,i}.ba;
    
    % incorporating Bias update
    phiDelta_bg = pre_imu{1,i}.dR_dbg*delta_bg;
    correctRMeas = pre_imu{1,i}.d_R*SO3.exp(phiDelta_bg);
    correctRMeas= renormalizeRotMat(correctRMeas);
    correctVMeas = pre_imu{1,i}.d_v + pre_imu{1,i}.dv_dba*delta_ba + pre_imu{1,i}.dv_dbg*delta_bg;
    correctPMeas = pre_imu{1,i}.d_p + pre_imu{1,i}.dp_dba*delta_ba + pre_imu{1,i}.dp_dbg*delta_bg;

    % residual error
    % [R, p, v, bg, ba]'
    % Residual for R
    err(1:3) = SO3.log(correctRMeas'*PVector.Rot{i}'*PVector.Rot{i+1});
%     correctRMeas'*PVector.Rot{i}'*PVector.Rot{i+1}
%     SO3.log(correctRMeas'*PVector.Rot{i}'*PVector.Rot{i+1})
%     rotm2eul(correctRMeas'*PVector.Rot{i}'*PVector.Rot{i+1})
    % Residual for v
    err(4:6) = PVector.Rot{i}'*(PVector.v(:,i+1) - PVector.v(:,i) - g*t) - correctVMeas;
    % Residual for p
    err(7:9) = PVector.Rot{i}'*(PVector.Pos(:,i+1) - PVector.Pos(:,i) - PVector.v(:,i)*t - g*t22) - correctPMeas;
    % Residual for bias_bg
    err(10:12) = PVector.bg(:,i+1) - PVector.bg(:,i);
    %Residual for bias_ba
    err(13:15) = PVector.ba(:,i+1) - PVector.ba(:,i);
    
    IMU_Error = [IMU_Error;err];
end