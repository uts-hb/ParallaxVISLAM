
function [Jacobian] = FuncJacobian(xVector,PVector,Feature,K,ID1,ID2,nJacobian,extrinsic)

nRowNumu = length(xVector.u);
Val = zeros(1,nJacobian);
aa = 1;

for j=1:nRowNumu/2;
    PID = xVector.PID(j,1);
    FID = xVector.FID(j,1);

    if PID == Feature(FID,3);
        P_I_i = PVector.Pos(:,PID);
        P_C_i = PVector.Pos(:,PID) + PVector.Rot{PID}*extrinsic(1:3,4); % CAM_Position: State vector is in IMU frame but I need Camera frame for PBA
        Fj = PVector.Feature(3*FID-2:3*FID-1,1);      
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        Xj = [sin(Fj(1))*cos(Fj(2));
              sin(Fj(2));
              cos(Fj(1))*cos(Fj(2))];        
        R_I_i = PVector.Rot{PID}; 
        R_C_i = PVector.Rot{PID}*extrinsic(1:3,1:3);     % CAM_rotation  

        x=K*R_C_i'*Xj;
        dudx = [1/x(3),0,-x(1)/x(3)^2;
                0,1/x(3),-x(2)/x(3)^2;];
        dxdRi = K*extrinsic(1:3,1:3)'*SO3.skew(R_I_i'*Xj); 
        dudRi = dudx*dxdRi; 
        dxdXj = K*R_C_i';
        dXjdPT = [cos(Fj(1))*cos(Fj(2)),-sin(Fj(1))*sin(Fj(2));
                  0,cos(Fj(2));
                  -sin(Fj(1))*cos(Fj(2)),-cos(Fj(1))*sin(Fj(2))];
        dudPT = dudx*dxdXj*dXjdPT;%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        Val(aa:aa+9) = [dudRi(1,:),dudRi(2,:),dudPT(1,:),dudPT(2,:)];
        aa = aa+10;

    elseif PID == Feature(FID,4);
        P_I_i = PVector.Pos(:,Feature(FID,3));
        P_C_i = PVector.Pos(:,Feature(FID,3)) + PVector.Rot{Feature(FID,3)}*extrinsic(1:3,4);
        P_I_i = PVector.Pos(:,PID);
        P_C_k = PVector.Pos(:,PID)+ PVector.Rot{PID}*extrinsic(1:3,4);
        Fj = PVector.Feature(3*FID-2:3*FID,1);        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        Xj = [sin(Fj(1))*cos(Fj(2));
            sin(Fj(2));
            cos(Fj(1))*cos(Fj(2))];
        R_I_i = PVector.Rot{Feature(FID,3)};
        R_C_i = PVector.Rot{Feature(FID,3)}*extrinsic(1:3,1:3); 

        R_I_k = PVector.Rot{PID}; 
        R_C_k = PVector.Rot{PID}*extrinsic(1:3,1:3); 

        Tik = P_C_k-P_C_i;
        Dik = norm(Tik);
        DotP = Xj'*Tik;
        Omega2 = acos(DotP/Dik);
        Xk = sin(Fj(3)+Omega2)*Dik*Xj-sin(Fj(3))*Tik;
        x=K*R_C_k'*Xk;
        dudx = [1/x(3),0,-x(1)/x(3)^2;
                0,1/x(3),-x(2)/x(3)^2;];
        dxdRk = K*extrinsic(1:3,1:3)'*SO3.skew(R_I_k'*Xk);
        dudRk = dudx*dxdRk; 
        dxdXk = K*R_C_k';
        dSinOdO = cos(Fj(3)+Omega2);
        dXkdO = dSinOdO*Dik*Xj-cos(Fj(3))*Tik;
        dudO = dudx*dxdXk*dXkdO;%%
        dSinOdO2 = dSinOdO;
        dO2dCosO2 = -1/sqrt(1-(DotP/Dik)^2);
        dDotPdTik = Xj';
        dDikdD2 = 1/(2*Dik);
        dD2dTik = 2*Tik';
        dTikdTi = -R_I_i;
        dTikdTk = R_I_k;
        dDotPdTi = dDotPdTik*dTikdTi;
        dDotPdTk = dDotPdTik*dTikdTk;
        dDikdTi = dDikdD2*dD2dTik*dTikdTi;
        dDikdTk = dDikdD2*dD2dTik*dTikdTk;
        dCosO2dTi = (dDotPdTi*Dik-dDikdTi*DotP)/Dik^2;
        dCosO2dTk = (dDotPdTk*Dik-dDikdTk*DotP)/Dik^2;
        dSinOdTi = dSinOdO2*dO2dCosO2*dCosO2dTi;
        dSinOdTk = dSinOdO2*dO2dCosO2*dCosO2dTk;
        dXkdTi = Xj*(dSinOdTi*Dik+dDikdTi*sin(Fj(3)+Omega2))-sin(Fj(3))*dTikdTi;
        dXkdTk = Xj*(dSinOdTk*Dik+dDikdTk*sin(Fj(3)+Omega2))-sin(Fj(3))*dTikdTk;
        dudTi = dudx*dxdXk*dXkdTi;%%
        dudTk = dudx*dxdXk*dXkdTk;%%
        dCosO2dXj = Tik'/Dik;
        dXjdPT = [cos(Fj(1))*cos(Fj(2)),-sin(Fj(1))*sin(Fj(2));
                  0,cos(Fj(2));
                  -sin(Fj(1))*cos(Fj(2)),-cos(Fj(1))*sin(Fj(2))];
        
        dSinOdPT = dSinOdO2*dO2dCosO2*dCosO2dXj*dXjdPT;
        dXkdPT = Dik*(Xj*dSinOdPT+sin(Fj(3)+Omega2)*dXjdPT);
        dudPT = dudx*dxdXk*dXkdPT;%%
        dudXj = cat(2,dudPT,dudO);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        Val(aa:aa+23) = [dudRk(1,:),dudRk(2,:),dudTi(1,:),dudTi(2,:),dudTk(1,:),dudTk(2,:),dudXj(1,:),dudXj(2,:)];
        aa = aa+24;
 
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
        R_I_i = PVector.Rot{Feature(FID,3)};
        R_C_i = PVector.Rot{Feature(FID,3)}*extrinsic(1:3,1:3);

        R_I_k = PVector.Rot{Feature(FID,4)};
        R_C_k = PVector.Rot{Feature(FID,4)}*extrinsic(1:3,1:3);

        R_I_l = PVector.Rot{PID};
        R_C_l = PVector.Rot{PID}*extrinsic(1:3,1:3);

        Tik = P_C_k-P_C_i;
        Til = P_C_l-P_C_i;
        Dik = norm(Tik);
        DotP = Xj'*Tik;
        Omega2 = acos(DotP/Dik);
        Xl = sin(Fj(3)+Omega2)*Dik*Xj-sin(Fj(3))*Til;
        x=K*R_C_l'*Xl;
        dudx = [1/x(3),0,-x(1)/x(3)^2;
                0,1/x(3),-x(2)/x(3)^2;];
        dxdRl = K*extrinsic(1:3,1:3)'*SO3.skew(R_I_l'*Xl);%%
        dudRl = dudx*dxdRl; 
        dxdXl = K*R_C_l';%%
        dSinOdO = cos(Fj(3)+Omega2);
        dXldO = dSinOdO*Dik*Xj-cos(Fj(3))*Til;
        dudO = dudx*dxdXl*dXldO;%%
        dSinOdO2 = dSinOdO;
        dO2dCosO2 = -1/sqrt(1-(DotP/Dik)^2);
        dDotPdTik = Xj';
        dDikdD2 = 1/(2*Dik);
        dD2dTik = 2*Tik';
        dTikdTi = -R_I_i;
        dTikdTk = R_I_k;
        dTildTi = -R_I_i;
        dTildTl = R_I_l;
        dDotPdTi = dDotPdTik*dTikdTi;
        dDotPdTk = dDotPdTik*dTikdTk;
        dDikdTi = dDikdD2*dD2dTik*dTikdTi;
        dDikdTk = dDikdD2*dD2dTik*dTikdTk;
        dCosO2dTi = (dDotPdTi*Dik-dDikdTi*DotP)/Dik^2;
        dCosO2dTk = (dDotPdTk*Dik-dDikdTk*DotP)/Dik^2;
        dSinOdTi = dSinOdO2*dO2dCosO2*dCosO2dTi;
        dSinOdTk = dSinOdO2*dO2dCosO2*dCosO2dTk;
        dXldTi = Xj*(dSinOdTi*Dik+dDikdTi*sin(Fj(3)+Omega2))-sin(Fj(3))*dTildTi;
        dXldTk = Xj*(dSinOdTk*Dik+dDikdTk*sin(Fj(3)+Omega2));
        dXldTl = -sin(Fj(3))*dTildTl;
        dudTi = dudx*dxdXl*dXldTi;%%
        dudTk = dudx*dxdXl*dXldTk;%%
        dudTl = dudx*dxdXl*dXldTl;%%
        dCosO2dXj = Tik'/Dik;
        dXjdPT = [cos(Fj(1))*cos(Fj(2)),-sin(Fj(1))*sin(Fj(2));
                  0,cos(Fj(2));
                  -sin(Fj(1))*cos(Fj(2)),-cos(Fj(1))*sin(Fj(2))];     
        dSinOdPT = dSinOdO2*dO2dCosO2*dCosO2dXj*dXjdPT;
        dXldPT = Dik*(Xj*dSinOdPT+sin(Fj(3)+Omega2)*dXjdPT);
        dudPT = dudx*dxdXl*dXldPT;%%
        dudXj = cat(2,dudPT,dudO);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
        Val(aa:aa+29) = [dudRl(1,:),dudRl(2,:),dudTi(1,:),dudTi(2,:),dudTk(1,:),dudTk(2,:),dudTl(1,:),dudTl(2,:),dudXj(1,:),dudXj(2,:)];
        aa = aa+30;
        
    end;
end;

Jacobian = sparse(ID1,ID2,Val);
