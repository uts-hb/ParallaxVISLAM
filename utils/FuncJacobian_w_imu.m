
function [Jacobian] = FuncJacobian_w_imu(xVector,PVector,Feature,K,ID1,ID2,nJacobian,extrinsic,pre_imu)

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

for i = 1 : size(PVector.Pos,2) - 1

    err = zeros(15,1);
    g = [0;0;-9.80655];

    t = sum(pre_imu{1, i}.dt);
    t22 = t*t/2;

    cov_b_ang = t*(0.0001^2)*eye(3,3);
    cov_b_acc = t*(0.001^2)*eye(3,3);

    delta_bg = PVector.bg(:,i+1) - pre_imu{1,i}.bg;
    delta_ba = PVector.ba(:,i+1) - pre_imu{1,i}.ba;

    % incorporating Bias update
    phiDelta_bg = pre_imu{1,i}.dR_dbg*delta_bg;
    correctRMeas = pre_imu{1,i}.d_R*SO3.exp(phiDelta_bg);
    correctRMeas= renormalizeRotMat(correctRMeas);
    correctVMeas = pre_imu{1,i}.d_v + pre_imu{1,i}.dv_dba*delta_ba + pre_imu{1,i}.dv_dbg*delta_bg;
    correctPMeas = pre_imu{1,i}.d_p + pre_imu{1,i}.dp_dba*delta_ba + pre_imu{1,i}.dp_dbg*delta_bg;

    % residual error besed on formula (37) in the paper
    % [R, p, v, bg, ba]'

    % Residual for R
    err(1:3) = SO3.log(correctRMeas'*PVector.Rot{i}'*PVector.Rot{i+1});
    % Residual for v
    err(4:6) = PVector.Rot{i}'*(PVector.v(:,i+1) - PVector.v(:,i) - g*t) - correctVMeas;
    % Residual for p
    err(7:9) = PVector.Rot{i}'*(PVector.Pos(:,i+1) - PVector.Pos(:,i) - PVector.v(:,i)*t - g*t22) - correctPMeas;
    % Residual for bias_bg
    err(10:12) = PVector.bg(:,i+1) - PVector.bg(:,i);
    %Residual for bias_ba
    err(13:15) = PVector.ba(:,i+1) - PVector.ba(:,i);

    Jr_Err_errPhiInv = SO3.Dlog(err(1:3));  %   Inverse of J_r
    RErrPhi = SO3.exp(err(1:3));            % Exp(residual_delta_R_ij)
    Jr_phiDelta_bg = SO3.Dexp(phiDelta_bg); %

    % Jacobian of Residual for R
    dR_dRi = -Jr_Err_errPhiInv*PVector.Rot{i+1}'*PVector.Rot{i}; % d resierror_deltaRij, d Ri
    dR_dbgi = -Jr_Err_errPhiInv*RErrPhi'*Jr_phiDelta_bg*pre_imu{1,i}.dR_dbg; % d resierror_deltaRij, d bg
    dR_dRj = Jr_Err_errPhiInv; % d resierror_deltaRij, d Rj
    dR_dbgj = -Jr_Err_errPhiInv*RErrPhi'*Jr_phiDelta_bg*pre_imu{1,i}.dR_dbg;

    J_R = [dR_dRi,dR_dRj,dR_dbgj];
    Val(aa:aa+26) = [J_R(1,:), J_R(2,:), J_R(3,:)]; 
    aa = aa + 27;


    % Jacobian of Residual for v
    dv_dRi = SO3.skew(PVector.Rot{i}'*(PVector.v(:,i+1) - PVector.v(:,i)  - g*t));
    dv_dvi = -PVector.Rot{i}';
    dv_dbgi = -pre_imu{1,i}.dv_dbg;     % d resierror_deltavij, d bg
    dv_dbai = -pre_imu{1,i}.dv_dba;     % d resierror_deltavij, d ba
    dv_dvj = PVector.Rot{i}';    % d resierror_deltavij, d vj
    dv_dbgj = -pre_imu{1,i}.dv_dbg;     % d resierror_deltavij, d bg
    dv_dbaj = -pre_imu{1,i}.dv_dba;     % d resierror_deltavij, d ba

    J_v = [dv_dRi,dv_dvi,dv_dvj,dv_dbgj,dv_dbaj];
    Val(aa:aa+44) = [J_v(1,:), J_v(2,:), J_v(3,:)]; 
    aa = aa + 45;

    % Jacobian of Residual for p
    dp_dRi= SO3.skew(PVector.Rot{i}'*(PVector.Pos(:,i+1)-PVector.Pos(:,i)- PVector.v(:,i)*t - g*t22));
    dp_dpi = -eye(3);
    dp_dvi= -PVector.Rot{i}' * t;
    dp_dbgi= -pre_imu{1,i}.dv_dbg;     % d resierror_deltavij, d ba
    dp_dbai= -pre_imu{1,i}.dv_dba;     % d resierror_deltavij, d bg
    dp_dpj = PVector.Rot{i}' * PVector.Rot{i+1};
    dp_dbgj= -pre_imu{1,i}.dv_dbg;     % d resierror_deltavij, d ba
    dp_dbaj= -pre_imu{1,i}.dv_dba;     % d resierror_deltavij, d bg

    J_p = [dp_dRi,dp_dpi,dp_dvi,dp_dpj,dp_dbgj,dp_dbaj];
    Val(aa:aa+53) = [J_p(1,:), J_p(2,:), J_p(3,:)]; 
    aa = aa + 54;

    % Jacobian of Residual for b_g
    dbg_dbgi = -eye(3); 
    dbg_dbgj = eye(3); 
    
    J_bg = [dbg_dbgi,dbg_dbgj];
    Val(aa:aa+17) = [J_bg(1,:), J_bg(2,:), J_bg(3,:)];
    aa = aa + 18;

    % Jacobian of Residual for b_a
    dba_dbai = -eye(3); 
    dba_dbaj = eye(3); 
    
    J_ba = [dba_dbai,dba_dbaj];
    Val(aa:aa+17) = [J_ba(1,:), J_ba(2,:), J_ba(3,:)];
    aa = aa + 18;



end 



Jacobian = sparse(ID1,ID2,Val);
