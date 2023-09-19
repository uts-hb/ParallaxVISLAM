
function [DeltaP,DeltaF,Sum_Delta] = FuncDeltaLMSBA_w_imu(Info,E,Lambda,PVector,Feature)

% nRowNumP = length(PVector.Pose);
nRowNumP = 15*size(PVector.Pos,2);
nRowNumI = nRowNumP+length(PVector.Feature);

C = sparse(1:nRowNumI,1:nRowNumI,1)*Lambda;
Info = Info+C;

U = Info(1:nRowNumP,1:nRowNumP);
V = Info(nRowNumP+1:nRowNumI,nRowNumP+1:nRowNumI);
W = Info(1:nRowNumP,nRowNumP+1:nRowNumI);
EP = E(1:nRowNumP);
EF = E(nRowNumP+1:nRowNumI);

[ID1,ID2,Val] = find(V);

i = 1;
nF = 1;

while i<=length(ID1);
        aa = reshape(Val(i:i+8),3,3);
        aa = inv(aa);
        Val(i:i+8) = reshape(aa,9,1);
        nF = nF+1;
        i = i+9;
end;

V = sparse(ID1,ID2,Val);
% clear ID1 ID2 Val;

S = U-W*V*W';
ES = EP-W*V*EF;

%%
S = S(16:nRowNumP,16:nRowNumP);
ES = ES(16:nRowNumP);
%%
DeltaP = S\ES;
%%
DeltaP = [zeros(15,1);DeltaP(1:nRowNumP-15)];
%%
DeltaF = V*(EF-W'*DeltaP);

Sum_Delta = DeltaP'*DeltaP+DeltaF'*DeltaF;

