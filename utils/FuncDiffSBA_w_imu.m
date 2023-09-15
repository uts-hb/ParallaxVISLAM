
function [PBA_Error,PBA_Sum_Error,IMU_Error, IMU_Sum_Error]= FuncDiffSBA_w_imu(xVector,PVector,Feature,K,extrinsic,pre_imu)

[uvcomp] = FuncfP(xVector,PVector,Feature,K,extrinsic);
PBA_Error = xVector.u-uvcomp;
PBA_Sum_Error = PBA_Error'*PBA_Error;

IMU_Error = -ErrorIMU(PVector, pre_imu);
IMU_Sum_Error = IMU_Error'*IMU_Error; 