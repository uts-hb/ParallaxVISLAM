function [PVector,Reason,Info,obj_fun,PBA_ERROR,IMU_ERROR] = FuncLeastSquares_w_IMU(xVector,PVector,Feature,K,extrinsic,pre_imu)

%Initial Error
obj_fun = []; 
PBA_ERROR = {}; 
IMU_ERROR = {};
Sum_Error = []; 
nRowNum = length(xVector.PID);
[errorPBA,sumErrorPBA,errorIMU,sumErrorIMU]= FuncDiff_w_imu(xVector,PVector,Feature,K,extrinsic,pre_imu);
% PBA_ERROR{end+1} = errorPBA; 
% IMU_ERROR{end+1} = errorIMU;
obj_fun = [obj_fun, [sumErrorPBA;sumErrorIMU]];
Error_PBA_IMU = [errorPBA;errorIMU];
Sum_Error = (sumErrorPBA+sumErrorIMU)/(nRowNum+size(PVector.Pos,2)-1);
sumErrorPBA = sumErrorPBA/(nRowNum);
sumErrorIMU = sumErrorIMU/(size(PVector.Pos,2)-1);
fprintf('PBA ERROR %.8f\n',sumErrorPBA);
fprintf('IMU ERROR %.8f\n',sumErrorIMU);
fprintf('Initial Error is %.8f\n', Sum_Error);
Sum_Delta = 22;
MaxIter = 50;
MinError = 1e-8;
MinDelta = 1e-4;
[ID1,ID2,nJacobian] = FuncGetJacobianID_w_imu(xVector,PVector,Feature);
W = funcWeight(xVector,PVector,pre_imu); 
Iter = 0;
while Sum_Error>MinError && Sum_Delta>MinDelta && Iter<=MaxIter;
    [Jacobian] = FuncJacobian_w_imu(xVector,PVector,Feature,K,ID1,ID2,nJacobian,extrinsic,pre_imu);
    [DeltaP,DeltaF,Sum_Delta] = FuncDelta_w_imu(Jacobian,Error_PBA_IMU,PVector,Feature(:,2),W);
    [PVector,pre_imu] = FuncUpdate_w_imu(PVector,DeltaP,DeltaF,pre_imu);
    [errorPBA,sumErrorPBA,errorIMU,sumErrorIMU]= FuncDiff_w_imu(xVector,PVector,Feature,K,extrinsic,pre_imu);
%     PBA_ERROR{end+1} = errorPBA;
%     IMU_ERROR{end+1} = errorIMU;
    obj_fun = [obj_fun, [sumErrorPBA;sumErrorIMU]];
    Error_PBA_IMU = [errorPBA;errorIMU];
    Sum_Error = (sumErrorPBA+sumErrorIMU)/(nRowNum+size(PVector.Pos,2)-1);
    sumErrorPBA = sumErrorPBA/(nRowNum);
    sumErrorIMU = sumErrorIMU/(size(PVector.Pos,2)-1);
    fprintf('PBA ERROR %.8f\n',sumErrorPBA);
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