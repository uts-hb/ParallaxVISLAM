
function [PVector,Feature,Reason,Info,obj_fun,SBA_ERROR,IMU_ERROR] = FuncLeastSquaresLMSBA_w_imu(uv,PVector,Feature,K,extrinsic,pre_imu)

nRowNum = size(uv,1)/2;
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
SBA_ERROR = {}; 
IMU_ERROR = {};
Sum_Error = []; 

[ID1,ID2,nJacobian] = FuncGetJacobianID_w_imu(uv,PVector,Feature);
[errorSBA,sumErrorSBA,errorIMU,sumErrorIMU]= FuncDiffSBA_w_imu(uv,PVector,Feature,K,extrinsic,pre_imu);
% SBA_ERROR{end+1} = errorSBA; 
% IMU_ERROR{end+1} = errorIMU;
Error_SBA_IMU = [errorSBA;errorIMU];
Error = Error_SBA_IMU'*Error_SBA_IMU;
Sum_Error = Error/(nRowNum+size(PVector.Pos,2)-1);
sumErrorSBA = sumErrorSBA/(nRowNum);
sumErrorIMU = sumErrorIMU/(size(PVector.Pos,2)-1);
obj_fun = [obj_fun, [sumErrorSBA;sumErrorIMU;Sum_Error]];
fprintf('SBA ERROR %.8f\n',sumErrorSBA);
fprintf('IMU ERROR %.8f\n',sumErrorIMU);
fprintf('Initial Error is %.8f\n', Sum_Error);
ErrorPre = Error;

[ID1,ID2,nJacobian] = FuncGetJacobianID_w_imu(uv,PVector,Feature);
[Jacobian] = FuncJacobian_w_imu(uv,PVector,Feature,K,ID1,ID2,nJacobian,extrinsic,pre_imu);
W = funcWeight(uv,PVector,pre_imu); 

A = Jacobian'*W*Jacobian;
Lambda = t*max(diag(A));
G = Jacobian'*W*Error_SBA_IMU;
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
        [PVector,pre_imu,Feature] = FuncUpdate_w_imu(PVector,DeltaP,DeltaF,pre_imu,Feature);
        [errorSBA,sumErrorSBA,errorIMU,sumErrorIMU]= FuncDiffSBA_w_imu(uv,PVector,Feature,K,extrinsic,pre_imu);
        Error_SBA_IMU = [errorSBA;errorIMU];
        Error = Error_SBA_IMU'*Error_SBA_IMU;
        Delta = [DeltaP;DeltaF];
        P = (ErrorPre-Error)/(Delta'*(Lambda*Delta+G));
        if P>0;
            if sqrt(ErrorPre)-sqrt(Error)<e4*sqrt(ErrorPre);
                Stop = 1;
                Reason = 3;
            end;
%             [Jacobian] = FuncJacobian(uv,PVector,Feature,K,ID1,ID2,nJacobian);
            [Jacobian] = FuncJacobian_w_imu(uv,PVector,Feature,K,ID1,ID2,nJacobian,extrinsic,pre_imu);
            A = Jacobian'*W*Jacobian;
            G = Jacobian'*W*Error_SBA_IMU;
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
            [PVector,pre_imu,Feature] = FuncUpdate_w_imu(PVector,DeltaP,DeltaF,pre_imu,Feature);
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
%         SBA_ERROR{end+1} = errorSBA;
%         IMU_ERROR{end+1} = errorIMU;
        sumErrorSBA = sumErrorSBA/(nRowNum);
        sumErrorIMU = sumErrorIMU/(size(PVector.Pos,2)-1);
        obj_fun = [obj_fun, [sumErrorSBA;sumErrorIMU;Sum_Error]];
        Sum_Error = Error/(nRowNum+size(PVector.Pos,2)-1);
        fprintf('SBA ERROR %.8f\n',sumErrorSBA);
        fprintf('IMU ERROR %.8f\n',sumErrorIMU);
        fprintf('Iterations %d Error %.8f\n', Iter,Sum_Error);
        ErrorPre = Error;
    end;
end;
    
Info = sparse([]);
if Iter>0;
    Info = Jacobian'*Jacobian;
end;