
function [ID1,ID2,nJacobian] = FuncGetJacobianID(xVector,PVector,Feature)

nRowNumu = length(xVector.u);
nRowNumP = 6*size(PVector.Pos,2);
ID1 = [];
ID2 = [];
nJacobian = 0;

for j=1:nRowNumu/2;
    PID = xVector.PID(j,1);
    FID = xVector.FID(j,1);
    
    if PID == Feature(FID,3);
        
        LID1 = [2*j-1,2*j-1,2*j-1,2*j,2*j,2*j,2*j-1,2*j-1,2*j,2*j];
        LID2 = [6*(PID-1)+1:6*(PID-1)+3,6*(PID-1)+1:6*(PID-1)+3,nRowNumP+3*FID-2:nRowNumP+3*FID-1,nRowNumP+3*FID-2:nRowNumP+3*FID-1];
        ID1 = [ID1,LID1];
        ID2 = [ID2,LID2];
        nJacobian = nJacobian+10;

    elseif PID == Feature(FID,4);
        
        LID1 = [2*j-1,2*j-1,2*j-1,2*j,2*j,2*j,2*j-1,2*j-1,2*j-1,2*j,2*j,2*j,2*j-1,2*j-1,2*j-1,2*j,2*j,2*j,2*j-1,2*j-1,2*j-1,2*j,2*j,2*j];
        LID2 = [6*(PID-1)+1:6*(PID-1)+3,6*(PID-1)+1:6*(PID-1)+3,6*(Feature(FID,3)-1)+4:6*(Feature(FID,3)-1)+6,6*(Feature(FID,3)-1)+4:6*(Feature(FID,3)-1)+6,6*(PID-1)+4:6*(PID-1)+6,6*(PID-1)+4:6*(PID-1)+6,nRowNumP+3*FID-2:nRowNumP+3*FID,nRowNumP+3*FID-2:nRowNumP+3*FID];
        ID1 = [ID1,LID1];
        ID2 = [ID2,LID2];
        nJacobian = nJacobian+24;
        
    else
        
        LID1 = [2*j-1,2*j-1,2*j-1,2*j,2*j,2*j,2*j-1,2*j-1,2*j-1,2*j,2*j,2*j,2*j-1,2*j-1,2*j-1,2*j,2*j,2*j,2*j-1,2*j-1,2*j-1,2*j,2*j,2*j,2*j-1,2*j-1,2*j-1,2*j,2*j,2*j];
        LID2 = [6*(PID-1)+1:6*(PID-1)+3,6*(PID-1)+1:6*(PID-1)+3,6*(Feature(FID,3)-1)+4:6*(Feature(FID,3)-1)+6,6*(Feature(FID,3)-1)+4:6*(Feature(FID,3)-1)+6,6*(Feature(FID,4)-1)+4:6*(Feature(FID,4)-1)+6,6*(Feature(FID,4)-1)+4:6*(Feature(FID,4)-1)+6,6*(PID-1)+4:6*(PID-1)+6,6*(PID-1)+4:6*(PID-1)+6,nRowNumP+3*FID-2:nRowNumP+3*FID,nRowNumP+3*FID-2:nRowNumP+3*FID];
        ID1 = [ID1,LID1];
        ID2 = [ID2,LID2];
        nJacobian = nJacobian+30;
        
    end;
end;

