function [PVector,Feature,Reason,Info,obj_fun,SBA_ERROR,IMU_ERROR] = FuncLeastSquares_w_IMU(uv,PVector,Feature,K,extrinsic,pre_imu)

%Initial Error
obj_fun = []; 
SBA_ERROR = {}; 
IMU_ERROR = {};
Sum_Error = []; 
nRowNum = size(uv,1)/2;
[errorSBA,sumErrorSBA,errorIMU,sumErrorIMU]= FuncDiff_w_imu(uv,PVector,Feature,K,extrinsic,pre_imu);
% SBA_ERROR{end+1} = errorSBA; 
% IMU_ERROR{end+1} = errorIMU;
obj_fun = [obj_fun, [sumErrorSBA;sumErrorIMU]];
Error_SBA_IMU = [errorSBA;errorIMU];
Sum_Error = (sumErrorSBA+sumErrorIMU)/(nRowNum+size(PVector.Pos,2)-1);
sumErrorSBA = sumErrorSBA/(nRowNum);
sumErrorIMU = sumErrorIMU/(size(PVector.Pos,2)-1);
fprintf('SBA ERROR %.8f\n',sumErrorSBA);
fprintf('IMU ERROR %.8f\n',sumErrorIMU);
fprintf('Initial Error is %.8f\n', Sum_Error);
Sum_Delta = 22;
MaxIter = 20;
MinError = 1e-8;
MinDelta = 1e-6;
[ID1,ID2,nJacobian] = FuncGetJacobianID_w_imu(uv,PVector,Feature);
W = funcWeight(uv,PVector,pre_imu); 
Iter = 0;
while Sum_Error>MinError && Sum_Delta>MinDelta && Iter<=MaxIter;
    [Jacobian] = FuncJacobian_w_imu(uv,PVector,Feature,K,ID1,ID2,nJacobian,extrinsic,pre_imu);
    [DeltaP,DeltaF,Sum_Delta] = FuncDelta_w_imu(Jacobian,Error_SBA_IMU,PVector,Feature(:,2),W);
    [PVector,pre_imu,Feature] = FuncUpdate_w_imu(PVector,DeltaP,DeltaF,pre_imu,Feature);
    [errorSBA,sumErrorSBA,errorIMU,sumErrorIMU]= FuncDiff_w_imu(uv,PVector,Feature,K,extrinsic,pre_imu);
%     SBA_ERROR{end+1} = errorSBA;
%     IMU_ERROR{end+1} = errorIMU;
    obj_fun = [obj_fun, [sumErrorSBA;sumErrorIMU]];
    Error_SBA_IMU = [errorSBA;errorIMU];
    Sum_Error = (sumErrorSBA+sumErrorIMU)/(nRowNum+size(PVector.Pos,2)-1);
    sumErrorSBA = sumErrorSBA/(nRowNum);
    sumErrorIMU = sumErrorIMU/(size(PVector.Pos,2)-1);
    fprintf('SBA ERROR %.8f\n',sumErrorSBA);
    fprintf('IMU ERROR %.8f\n',sumErrorIMU);
    Iter = Iter+1;
    fprintf('Iterations %d Error %.8f\n', Iter,Sum_Error);
end;

if Sum_Error<MinError;
    Reason = 1;
elseif Sum_Delta<MinDelta;
    Reason = 2;
elseif Iter>MaxIter;
    Reason = 3;
else
    Reason = 4;
end;

Info = sparse([]);
if Iter>0;
    Info = Jacobian'*W*Jacobian;
end;