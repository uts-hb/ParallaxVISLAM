
function [PVector,Reason,Info,obj_fun,PBA_ERROR,IMU_ERROR] = FuncLeastSquaresLMSBA_w_imu(xVector,PVector,Feature,K,extrinsic,pre_imu)

nRowNum = length(xVector.PID);
MaxIter = 5000;
Factor = 2;
t = 1e-8;
e1 = 1e-10;
e2 = 1e-10;
% e2 = 0; 
e3 = 1e-10;
e4 = 0;
Stop = 0;
Reason = 0;
Iter = 0;
obj_fun = []; 
PBA_ERROR = {}; 
IMU_ERROR = {};
Sum_Error = []; 

[ID1,ID2,nJacobian] = FuncGetJacobianID_w_imu(xVector,PVector,Feature);
[errorPBA,sumErrorPBA,errorIMU,sumErrorIMU]= FuncDiffSBA_w_imu(xVector,PVector,Feature,K,extrinsic,pre_imu);
% PBA_ERROR{end+1} = errorPBA; 
% IMU_ERROR{end+1} = errorIMU;
Error_PBA_IMU = [errorPBA;errorIMU];
Error = Error_PBA_IMU'*Error_PBA_IMU;
Sum_Error = Error/(nRowNum+size(PVector.Pos,2)-1);
sumErrorPBA = sumErrorPBA/(nRowNum);
sumErrorIMU = sumErrorIMU/(size(PVector.Pos,2)-1);
obj_fun = [obj_fun, [sumErrorPBA;sumErrorIMU;Sum_Error]];
fprintf('PBA ERROR %.8f\n',sumErrorPBA);
fprintf('IMU ERROR %.8f\n',sumErrorIMU);
fprintf('Initial Error is %.8f\n', Sum_Error);
ErrorPre = Error;

[ID1,ID2,nJacobian] = FuncGetJacobianID_w_imu(xVector,PVector,Feature);
[Jacobian] = FuncJacobian_w_imu(xVector,PVector,Feature,K,ID1,ID2,nJacobian,extrinsic,pre_imu);
W = funcWeight(xVector,PVector,pre_imu); 

A = Jacobian'*W*Jacobian;
Lambda = t*max(diag(A));
G = Jacobian'*W*Error_PBA_IMU;
g = max(abs(G));

if g<=e1;
    Stop = 1;
    Reason = 1;
end;

while Stop~=1 && Iter<=MaxIter;
    Iter = Iter+1;
    P = -1;
    while Stop~=1 && P<=0;
    [DeltaP,DeltaF,Sum_Delta] = FuncDeltaLMSBA_w_imu(A,G,Lambda,PVector,Feature(:,2));
    P2 = FuncGetP2_w_imu(PVector,extrinsic);
    if Sum_Delta<=e2*(P2+e2);
        Stop = 1;
        Reason = 2;
    else
        [PVector,pre_imu] = FuncUpdate_w_imu(PVector,DeltaP,DeltaF,pre_imu);
        [errorPBA,sumErrorPBA,errorIMU,sumErrorIMU]= FuncDiffSBA_w_imu(xVector,PVector,Feature,K,extrinsic,pre_imu);
        Error_PBA_IMU = [errorPBA;errorIMU];
        Error = Error_PBA_IMU'*Error_PBA_IMU;
        Delta = [DeltaP;DeltaF];
        P = (ErrorPre-Error)/(Delta'*(Lambda*Delta+G));
        if P>0;
            if sqrt(ErrorPre)-sqrt(Error)<e4*sqrt(ErrorPre);
                Stop = 1;
                Reason = 3;
            end;
%             [Jacobian] = FuncJacobian(xVector,PVector,Feature,K,ID1,ID2,nJacobian);
            [Jacobian] = FuncJacobian_w_imu(xVector,PVector,Feature,K,ID1,ID2,nJacobian,extrinsic,pre_imu);
            A = Jacobian'*W*Jacobian;
            G = Jacobian'*W*Error_PBA_IMU;
            g = max(abs(G));
            if Stop ==1 || g<=e1;
                Stop = 1;
                Reason = 1;
            end;
            Lambda = Lambda*max(1/3,1-(2*P-1)^3);
            Factor = 2;
        else    
            DeltaP = -DeltaP;
            DeltaF = -DeltaF;
            [PVector,pre_imu] = FuncUpdate_w_imu(PVector,DeltaP,DeltaF,pre_imu);
            Lambda = Factor*Lambda;
            Factor = Factor*2;
        end;
    end;
    end;
    if sqrt(Sum_Error)<=e3;
        Stop = 1;
        Reason = 4;
    end;
    if P>0;
%         PBA_ERROR{end+1} = errorPBA;
%         IMU_ERROR{end+1} = errorIMU;
        sumErrorPBA = sumErrorPBA/(nRowNum);
        sumErrorIMU = sumErrorIMU/(size(PVector.Pos,2)-1);
        obj_fun = [obj_fun, [sumErrorPBA;sumErrorIMU;Sum_Error]];
        Sum_Error = Error/(nRowNum+size(PVector.Pos,2)-1);
        fprintf('PBA ERROR %.8f\n',sumErrorPBA);
        fprintf('IMU ERROR %.8f\n',sumErrorIMU);
        fprintf('Iterations %d Error %.8f\n', Iter,Sum_Error);
        ErrorPre = Error;
    end;
end;
    
Info = sparse([]);
if Iter>0;
    Info = Jacobian'*Jacobian;
end;