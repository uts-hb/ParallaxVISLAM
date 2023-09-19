
function [PVector,Reason,Info,obj_fun,PBA_ERROR] = FuncLeastSquaresLMSBA(xVector,PVector,Feature,K,extrinsic,FixVa)

obj_fun = []; 
PBA_ERROR = {}; 

nRowNum = length(xVector.PID);
MaxIter = 50;
Factor = 2;
t = 1e-10;
% t = 1e-12;
e1 = 1e-12;
e2 = 1e-12;
% e2 = 0; 
e3 = 1e-12;
e4 = 0;
Stop = 0;
Reason = 0;
Iter = 0;

[errorPBA,sumErrorPBA]= FuncDiffSBA(xVector,PVector,Feature,K,extrinsic);
PBA_ERROR{end+1} = errorPBA; 
ErrorPre = sumErrorPBA;
sumErrorPBA = sumErrorPBA/(nRowNum);
obj_fun = [obj_fun, sumErrorPBA];
fprintf('Initial Error is %.8f\n', sumErrorPBA);

[ID1,ID2,nJacobian] = FuncGetJacobianID(xVector,PVector,Feature);
[Jacobian] = FuncJacobian(xVector,PVector,Feature,K,ID1,ID2,nJacobian,extrinsic);
A = Jacobian'*Jacobian;
Lambda = t*max(diag(A));
G = Jacobian'*errorPBA;
g = max(abs(G));

if g<=e1;
    Stop = 1;
    Reason = 1;
end;

while Stop~=1 && Iter<=MaxIter;
    Iter = Iter+1;
    P = -1;
    while Stop~=1 && P<=0;
    [DeltaP,DeltaF,Sum_Delta] = FuncDeltaLMSBA(A,G,Lambda,PVector,Feature(:,2),FixVa);
     P2 = FuncGetP2(PVector,extrinsic);
    if Sum_Delta<=e2*(P2+e2);
        Stop = 1;
        Reason = 2;
    else
        [PVector] = FuncUpdate(PVector,DeltaP,DeltaF,FixVa);
        [errorPBA,sumErrorPBA]= FuncDiffSBA(xVector,PVector,Feature,K,extrinsic);
        Delta = [DeltaP;DeltaF];
        P = (ErrorPre-sumErrorPBA)/(Delta'*(Lambda*Delta+G));
        if P>0;
            if sqrt(ErrorPre)-sqrt(sumErrorPBA)<e4*sqrt(ErrorPre);
                Stop = 1;
                Reason = 3;
            end;
            [Jacobian] = FuncJacobian(xVector,PVector,Feature,K,ID1,ID2,nJacobian,extrinsic);
            A = Jacobian'*Jacobian;
            G = Jacobian'*errorPBA;
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
            [PVector] = FuncUpdate(PVector,DeltaP,DeltaF,FixVa);
            Lambda = Factor*Lambda;
            Factor = Factor*2;
        end;
    end;
    end;
    if sqrt(sumErrorPBA)<=e3;
        Stop = 1;
        Reason = 4;
    end;
    if P>0;

        PBA_ERROR{end+1} = errorPBA; 
        ErrorPre = sumErrorPBA;
        sumErrorPBA = sumErrorPBA/(nRowNum);
        obj_fun = [obj_fun, sumErrorPBA];
        fprintf('Iterations %d Error %.8f\n', Iter, sumErrorPBA);     
    end;
end;
    
Info = sparse([]);
if Iter>0;
    Info = Jacobian'*Jacobian;
end;