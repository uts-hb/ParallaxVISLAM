
function [SBA_Error,SBA_Sum_Error,IMU_Error, IMU_Sum_Error]= FuncDiffSBA_w_imu(uv,PVector,Feature,K,extrinsic,pre_imu)

[uvcomp] = FuncfP(PVector,Feature,K,extrinsic);
SBA_Error = uv-uvcomp;
SBA_Sum_Error = SBA_Error'*SBA_Error;

IMU_Error = -ErrorIMU(PVector, pre_imu);
IMU_Sum_Error = IMU_Error'*IMU_Error; 