
function [PVector,pre_imu] = FuncUpdate_w_imu(PVector,DeltaP,DeltaF,pre_imu)

% PVector.Pose = PVector.Pose+DeltaP;
for i = 1 : size(PVector.Pos,2)

    R = PVector.Rot{i};
    delta_phi = DeltaP(15*(i-1)+1:15*(i-1)+3,1);
    p = PVector.Pos(:,i);
    delta_p = DeltaP(15*(i-1)+4:15*(i-1)+6,1);
    v = PVector.v(:,i);
    delta_v = DeltaP(15*(i-1)+7:15*(i-1)+9,1);
    b_g = PVector.bg(:,i);
    delta_bg = DeltaP(15*(i-1)+10:15*(i-1)+12,1);
    b_a = PVector.ba(:,i);
    delta_ba = DeltaP(15*(i-1)+13:15*(i-1)+15,1);

    PVector.Rot{i} = R * SO3.exp(delta_phi); 
%     PVector.Rot{i} = renormalizeRotMat(PVector.Rot{i});
    PVector.Pos(:,i) = p + R*delta_p; 
    PVector.v(:,i) = v + delta_v; 
    PVector.bg(:,i) = b_g + delta_bg; 
    PVector.ba(:,i) = b_a + delta_ba; 


%     if i == 2 
%         delP = R*delta_p; 
%         if FixVa==3;%% Fix Z
%             delP(3) = 0; 
%         elseif FixVa==2;%% Fix Y
%             delP(2) = 0;
%         elseif FixVa==1;%% Fix X
%             delP(1) = 0;
%         end
%         PVector.Pos(:,i) = p + delP; 
%     else
%         PVector.Pos(:,i) = p + R*delta_p; 
%     end 

end 

    for i = 2:size(PVector.Pos,2)-1
        PVector.bg(:,i) = PVector.bg(:,i+1);
        PVector.ba(:,i) = PVector.ba(:,i+1);

%         pre_imu{1,i-1}.bg = state_vector{1,i}.bg;
%         pre_imu{1,i-1}.ba = state_vector{1,i}.ba;
    end


PVector.Feature = PVector.Feature+DeltaF;


