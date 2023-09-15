
function [uvcomp] = FuncfP(xVector,PVector,Feature,K,extrinsic)

nRowNum = length(xVector.u);
uvcomp = zeros(nRowNum,1);

for j=1:nRowNum/2;
    PID = xVector.PID(j,1);
    FID = xVector.FID(j,1);

    if PID == Feature(FID,3);
        P_I_i = PVector.Pos(:,PID);
        P_C_i = PVector.Pos(:,PID) + PVector.Rot{PID}*extrinsic(1:3,4);
        Fj = PVector.Feature(3*FID-2:3*FID-1,1);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        Xj = [sin(Fj(1))*cos(Fj(2));
            sin(Fj(2));
            cos(Fj(1))*cos(Fj(2))];
        R_I_i = PVector.Rot{PID}; 
        R_C_i = PVector.Rot{PID}*extrinsic(1:3,1:3);     % CAM_rotation  
        x=K*R_C_i'*Xj;
        u = [x(1)/x(3);
            x(2)/x(3)];
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        uvcomp(2*j-1:2*j) = u;

    elseif PID == Feature(FID,4);
        P_I_i = PVector.Pos(:,Feature(FID,3));
        P_C_i = PVector.Pos(:,Feature(FID,3)) + PVector.Rot{Feature(FID,3)}*extrinsic(1:3,4);
        P_I_i = PVector.Pos(:,PID);
        P_C_k = PVector.Pos(:,PID)+ PVector.Rot{PID}*extrinsic(1:3,4);
        Fj = PVector.Feature(3*FID-2:3*FID,1);      
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        Xj = [sin(Fj(1))*cos(Fj(2));
              sin(Fj(2));
              cos(Fj(1))*cos(Fj(2))];  
        R_I_k = PVector.Rot{PID}; 
        R_C_k = PVector.Rot{PID}*extrinsic(1:3,1:3); 

        Tik = P_C_k-P_C_i;
        Dik = norm(Tik);
        Omega2 = acos(Xj'*Tik/Dik);
        Xk = sin(Fj(3)+Omega2)*Dik*Xj-sin(Fj(3))*Tik;
        x=K*R_C_k'*Xk;
        u = [x(1)/x(3);
            x(2)/x(3)];
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        uvcomp(2*j-1:2*j) = u;

    else
        P_I_i = PVector.Pos(:,Feature(FID,3));
        P_C_i = PVector.Pos(:,Feature(FID,3)) + PVector.Rot{Feature(FID,3)}*extrinsic(1:3,4); 
        P_I_k = PVector.Pos(:,Feature(FID,4));
        P_C_k = PVector.Pos(:,Feature(FID,4)) + PVector.Rot{Feature(FID,4)}*extrinsic(1:3,4); 
        P_I_l = PVector.Pos(:,PID);
        P_C_l = PVector.Pos(:,PID)+ PVector.Rot{PID}*extrinsic(1:3,4);
        Fj = PVector.Feature(3*FID-2:3*FID,1);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        Xj = [sin(Fj(1))*cos(Fj(2));
            sin(Fj(2));
            cos(Fj(1))*cos(Fj(2))];
        
        R_I_l = PVector.Rot{PID};
        R_C_l = PVector.Rot{PID}*extrinsic(1:3,1:3);

        Tik = P_C_k-P_C_i;
        Dik = norm(Tik);
        Omega2 = acos(Xj'*Tik/Dik);
        Xl = sin(Fj(3)+Omega2)*Dik*Xj-sin(Fj(3))*(P_C_l-P_C_i);
        x=K*R_C_l'*Xl;
        u = [x(1)/x(3);
            x(2)/x(3)];
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        uvcomp(2*j-1:2*j) = u;

    end;
end;

