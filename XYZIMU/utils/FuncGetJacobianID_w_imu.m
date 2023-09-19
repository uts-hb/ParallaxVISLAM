
function [ID1,ID2,nJacobian] = FuncGetJacobianID_w_imu(uv,PVector,Feature)

nRowNumu = size(uv,1);
nRowNumP = 15*size(PVector.Pos,2);

ID1 = [];
ID2 = [];
nJacobian = 0;

uvN = 1; 

for i=1:size(Feature,1)
    for j = 1 : Feature(i,4)
        
        LID1 = [(2*uvN-1)*ones(1,9),(2*uvN)*ones(1,9)];
%         LID1 = [2*Feature(5+3*(j-1))];
%         LID1 = [(2*uvN-1),(2*uvN-1),(2*uvN-1),(2*uvN-1),(2*uvN-1),(2*uvN-1),(2*uvN),(2*uvN),(2*uvN),(2*uvN),(2*uvN),(2*uvN),(2*uvN-1),(2*uvN-1),(2*uvN-1),(2*uvN),(2*uvN),(2*uvN)]; 
        LID2 = [15*(Feature(i,5+3*(j-1)))+1:15*(Feature(i,5+3*(j-1)))+6, nRowNumP+3*i-2:nRowNumP+3*i,15*(Feature(i,5+3*(j-1)))+1:15*(Feature(i,5+3*(j-1)))+6, nRowNumP+3*i-2:nRowNumP+3*i];
        ID1 = [ID1, LID1];
        ID2 = [ID2, LID2]; 
        nJacobian = nJacobian + 18;

        uvN = uvN + 1; 

    end 
end

for i = 1 : size(PVector.Pos,2)-1

        % Jacobian of Residual for R
        LID1 = [(nRowNumu+(15*(i-1)+1))*ones(1,9), (nRowNumu+(15*(i-1)+2))*ones(1,9), (nRowNumu+(15*(i-1)+3))*ones(1,9)];
        LID2 = [(15*(i-1)+1):(15*(i-1)+3),(15*(i-1)+16):(15*(i-1)+18),(15*(i-1)+25):(15*(i-1)+27),(15*(i-1)+1):(15*(i-1)+3),(15*(i-1)+16):(15*(i-1)+18),(15*(i-1)+25):(15*(i-1)+27),(15*(i-1)+1):(15*(i-1)+3),(15*(i-1)+16):(15*(i-1)+18),(15*(i-1)+25):(15*(i-1)+27)];
        ID1 = [ID1,LID1];
        ID2 = [ID2,LID2];
        nJacobian = nJacobian+27;

        % Jacobian of Residual for v
        LID1 = [(nRowNumu+(15*(i-1)+4))*ones(1,15), (nRowNumu+(15*(i-1)+5))*ones(1,15), (nRowNumu+(15*(i-1)+6))*ones(1,15)];
        LID2 = [(15*(i-1)+1):(15*(i-1)+3),(15*(i-1)+7):(15*(i-1)+9),(15*(i-1)+22):(15*(i-1)+24),(15*(i-1)+25):(15*(i-1)+27), (15*(i-1)+28):(15*(i-1)+30),(15*(i-1)+1):(15*(i-1)+3),(15*(i-1)+7):(15*(i-1)+9),(15*(i-1)+22):(15*(i-1)+24),(15*(i-1)+25):(15*(i-1)+27), (15*(i-1)+28):(15*(i-1)+30),(15*(i-1)+1):(15*(i-1)+3),(15*(i-1)+7):(15*(i-1)+9),(15*(i-1)+22):(15*(i-1)+24),(15*(i-1)+25):(15*(i-1)+27),(15*(i-1)+28):(15*(i-1)+30)];
        ID1 = [ID1,LID1];
        ID2 = [ID2,LID2];
        nJacobian = nJacobian+45;

        % Jacobian of Residual for p
        LID1 = [(nRowNumu+(15*(i-1)+7))*ones(1,18), (nRowNumu+(15*(i-1)+8))*ones(1,18), (nRowNumu+(15*(i-1)+9))*ones(1,18)];
        LID2 = [(15*(i-1)+1):(15*(i-1)+3), (15*(i-1)+4):(15*(i-1)+6), (15*(i-1)+7):(15*(i-1)+9), (15*(i-1)+19):(15*(i-1)+21),(15*(i-1)+25):(15*(i-1)+27),(15*(i-1)+28):(15*(i-1)+30),(15*(i-1)+1):(15*(i-1)+3), (15*(i-1)+4):(15*(i-1)+6), (15*(i-1)+7):(15*(i-1)+9), (15*(i-1)+19):(15*(i-1)+21),(15*(i-1)+25):(15*(i-1)+27),(15*(i-1)+28):(15*(i-1)+30),(15*(i-1)+1):(15*(i-1)+3), (15*(i-1)+4):(15*(i-1)+6), (15*(i-1)+7):(15*(i-1)+9), (15*(i-1)+19):(15*(i-1)+21),(15*(i-1)+25):(15*(i-1)+27),(15*(i-1)+28):(15*(i-1)+30)];
        ID1 = [ID1,LID1];
        ID2 = [ID2,LID2];
        nJacobian = nJacobian+54;


        % Jacobian of Residual for bg 
        LID1 = [(nRowNumu+(15*(i-1)+10))*ones(1,6), (nRowNumu+(15*(i-1)+11))*ones(1,6), (nRowNumu+(15*(i-1)+12))*ones(1,6)];
        LID2 = [(15*(i-1)+10):(15*(i-1)+12),(15*(i-1)+25):(15*(i-1)+27),(15*(i-1)+10):(15*(i-1)+12),(15*(i-1)+25):(15*(i-1)+27),(15*(i-1)+10):(15*(i-1)+12),(15*(i-1)+25):(15*(i-1)+27)];
        ID1 = [ID1,LID1];
        ID2 = [ID2,LID2];
        nJacobian = nJacobian+18;

        % Jacobian of Residual for ba
        LID1 = [(nRowNumu+(15*(i-1)+13))*ones(1,6), (nRowNumu+(15*(i-1)+14))*ones(1,6), (nRowNumu+(15*(i-1)+15))*ones(1,6)];
        LID2 = [(15*(i-1)+13):(15*(i-1)+15),(15*(i-1)+28):(15*(i-1)+30),(15*(i-1)+13):(15*(i-1)+15),(15*(i-1)+28):(15*(i-1)+30),(15*(i-1)+13):(15*(i-1)+15),(15*(i-1)+28):(15*(i-1)+30)];
        ID1 = [ID1,LID1];
        ID2 = [ID2,LID2];
        nJacobian = nJacobian+18;

end 

