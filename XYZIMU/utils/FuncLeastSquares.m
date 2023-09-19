
function [PVector,Reason,Info,obj_fun,PBA_ERROR] = FuncLeastSquares(xVector,PVector,Feature,K,extrinsic,FixVa)

obj_fun = []; 
PBA_ERROR = {}; 
%Initial Error for PBA
nRowNum = length(xVector.PID);
[errorPBA, sumErrorPBA]= FuncDiff(xVector,PVector,Feature,K,extrinsic);
PBA_ERROR{end+1} = errorPBA; 
sumErrorPBA = sumErrorPBA/nRowNum;
obj_fun = [obj_fun, sumErrorPBA];
fprintf('Initial Error is %.8f\n', sumErrorPBA);
Sum_Delta = 22;
MaxIter = 50;
MinError = 1e-8;
MinDelta = 1e-10;

[ID1,ID2,nJacobian] = FuncGetJacobianID(xVector,PVector,Feature);

Iter = 0;
while sumErrorPBA>MinError && Sum_Delta>MinDelta && Iter<=MaxIter;
    [Jacobian] = FuncJacobian(xVector,PVector,Feature,K,ID1,ID2,nJacobian,extrinsic);
    [DeltaP,DeltaF,Sum_Delta] = FuncDelta(Jacobian,errorPBA,PVector,Feature(:,2),FixVa);
    [PVector] = FuncUpdate(PVector,DeltaP,DeltaF);
    [errorPBA, sumErrorPBA]= FuncDiff(xVector,PVector,Feature,K,extrinsic);
    PBA_ERROR{end+1} = errorPBA;
    obj_fun = [obj_fun, sumErrorPBA];
    sumErrorPBA = sumErrorPBA/nRowNum;
    fprintf('Iterations %d Error %.8f\n', Iter,sumErrorPBA);
    Iter = Iter+1;
end;

if sumErrorPBA<MinError;
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
    Info = Jacobian'*Jacobian;
end;