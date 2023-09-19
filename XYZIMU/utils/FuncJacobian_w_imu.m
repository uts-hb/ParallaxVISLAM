
function [Jacobian] = FuncJacobian_w_imu(uv,PVector,Feature,K,ID1,ID2,nJacobian,extrinsic,pre_imu)

nRowNumu = size(uv,1);
Val = zeros(1,nJacobian);
aa = 1;

for i=1:size(Feature,1)
    Xj = Feature(i,1:3);
    for j = 1 : Feature(i,4)


        po = Feature(i,5+3*(j-1))+1;

        R_imu = PVector.Rot{1,po};
        t_imu = PVector.Pos(:,po);
        R_cam = R_imu*extrinsic(1:3,1:3); 
        t_cam = t_imu + R_imu*extrinsic(1:3,4);

        x = K*R_cam'*(Xj(1:3)'-t_cam);
        dudx = [1/x(3),0,-x(1)/x(3)^2;
                0,1/x(3),-x(2)/x(3)^2;];
        dxdRi = K*extrinsic(1:3,1:3)'*SO3.skew(R_imu'*(Xj(1:3)'-t_imu));
        dudRi = dudx*dxdRi;

        dxdTi = -K*R_cam'*R_imu; 
        dudTi = dudx*dxdTi; 

%         x = K*R_cam'*(Xj(1:3)'-t_cam);
%         dudx = [1/x(3),0,-x(1)/x(3)^2;
%                 0,1/x(3),-x(2)/x(3)^2;];
%         dxdRcam = K*SO3.skew(R_cam'*(Xj(1:3)'-t_cam));
%         dRcamdRimu = extrinsic(1:3,1:3);
%         dudRi = dudx*dxdRcam*dRcamdRimu; 
% 
%         dxdPTc = -K*R_cam';
%         dudPT = dudx*dxdPTc; 
% 
% 
        dxdXj = K*R_cam';
        dudXj = dudx*dxdXj;
% 
        dudX = [dudRi, dudTi, dudXj]; 
%         dudP = dudX(1,:);
%         dvdP = dudX(2,:);
% %         
% 
% 
    Val(aa:aa+17) = [dudX(1,:),dudX(2,:)];
        aa = aa + 18;     
%         

   
    
    end
end

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
