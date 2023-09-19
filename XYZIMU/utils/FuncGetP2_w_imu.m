function P2 = FuncGetP2_w_imu(PVector,extrinsic)

Pose = []; 
for i = 1 : size(PVector.Pos,2)
    R = rotm2eul(PVector.Rot{i})';
    p = PVector.Pos(:,i); 
    Pose = [Pose;R;p]; 
end 

%%
PP1 = Pose(7:9)'*Pose(7:9);
PP2 = Pose(11:end)'*Pose(11:end);
%%
PP3 = PVector.Feature'*PVector.Feature;
P2 = sqrt(PP1+PP2+PP3);