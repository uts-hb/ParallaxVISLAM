
function [PBA_Error,PBA_Sum_Error]= FuncDiffSBA(xVector,PVector,Feature,K,extrinsic)

[uvcomp] = FuncfP(xVector,PVector,Feature,K,extrinsic);
PBA_Error = xVector.u-uvcomp;
PBA_Sum_Error = PBA_Error'*PBA_Error;
