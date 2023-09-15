
function [PBA_Error, PBA_Sum_Error]= FuncDiff(xVector,PVector,Feature,K,extrinsic)

[uvcomp] = FuncfP(xVector,PVector,Feature,K,extrinsic);
PBA_Error = uvcomp-xVector.u;
PBA_Sum_Error = PBA_Error'*PBA_Error;


