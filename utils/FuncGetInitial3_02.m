function [PVector,Feature] = FuncGetInitial3_02(PVector,Feature,Image,PoseID,K,extrinsic,start_ImageNum)

% Feature matrix column (1) FeatureID, (2) if 2 only once, if 3 observed
% twice, (3) Main Anchor id (4) Asscicated Anchor id (5) number of Feature
% observed 


PVector.Rot{end+1} = eul2rotm(Image(1,1:3));
PVector.Pos = [PVector.Pos,Image(1,4:6)'];

[nRowNum,nColumNum] = size(Image);
[nRowNumF,nColumNumF] = size(Feature);
 %ID,kind(2or3),M_anchor,A_anchor,Times,Pose_i,uv,...
for i=1:nRowNum-1;
    FID = Image(i+1,1);
    if FID > nRowNumF;
        Feature(FID,1) = FID;
        Feature(FID,2) = 2;
        Feature(FID,3) = PoseID-(start_ImageNum-1);
        Feature(FID,5) = 1;
        Feature(FID,6) = PoseID-(start_ImageNum-1);
        Feature(FID,7:8) = Image(i+1,2:3);
        V1 = Funcuv2V(Image(i+1,2:3),PVector.Rot{1,PoseID-(start_ImageNum-1)},K,extrinsic);
%         V1 = Funcuv2V(Image(i+1,2:3),PVector.Pose(6*(PoseID-1)+1:6*(PoseID-1)+3,1),K,extrinsic);
        [PVector.Feature(3*FID-2,1),PVector.Feature(3*FID-1,1)] = FuncV2PT(V1);
%         clear V1;
        continue;
        
    elseif Feature(FID,5) == 0;
        Feature(FID,1) = FID;
        Feature(FID,2) = 2;
        Feature(FID,3) = PoseID-(start_ImageNum-1);
        Feature(FID,5) = 1;
        Feature(FID,6) = PoseID-(start_ImageNum-1);
        Feature(FID,7:8) = Image(i+1,2:3);
        V1 = Funcuv2V(Image(i+1,2:3),PVector.Rot{1,PoseID-(start_ImageNum-1)},K,extrinsic);
        [PVector.Feature(3*FID-2,1),PVector.Feature(3*FID-1,1)] = FuncV2PT(V1);
%         clear V1;
        continue;
    
    elseif Feature(FID,5) == 1;
        Feature(FID,2) = 3;
        Feature(FID,4) = PoseID-(start_ImageNum-1);
        Feature(FID,5) = 2;
        Feature(FID,9) = PoseID-(start_ImageNum-1);
        Feature(FID,10:11) = Image(i+1,2:3);
        V1 = Funcuv2V(Feature(FID,7:8),PVector.Rot{1,Feature(FID,3)},K,extrinsic);
        V2 = Funcuv2V(Image(i+1,2:3),PVector.Rot{1,PoseID-(start_ImageNum-1)},K,extrinsic);
        PVector.Feature(3*FID,1) = FuncV2O(V1,V2);
%         clear V1 V2;
        continue;
    
    elseif Feature(FID,5) >= 2;
        Omegai = [];
%         if PVector.Feature(3*FID,1)<0; 
        if PVector.Feature(3*FID,1)<0.5;   
            V2 = Funcuv2V(Image(i+1,2:3),PVector.Rot{1,PoseID-(start_ImageNum-1)},K,extrinsic);
            for j=1:Feature(FID,5); 
                V1 = Funcuv2V(Feature(FID,3*(j-1)+7:3*(j-1)+8),PVector.Rot{1,Feature(FID,3*(j-1)+6)},K,extrinsic);
%                 V1 = Funcuv2V(Feature(FID,3*(j-1)+7:3*(j-1)+8),PVector.Pose(6*(Feature(FID,3*(j-1)+6)-1)+1:6*(Feature(FID,3*(j-1)+6)-1)+3),K,extrinsic);
                Omegai(j) = FuncV2O(V1,V2);
            end;
            [Max,ID] = max(Omegai);
            if Max>PVector.Feature(3*FID,1)*1.2 || Max>0.5;
                PVector.Feature(3*FID,1) = Max;
                Feature(FID,3) = Feature(FID,3*(ID-1)+6);
                Feature(FID,4) = PoseID-(start_ImageNum-1);
                V1 = Funcuv2V(Feature(FID,3*(ID-1)+7:3*(ID-1)+8),PVector.Rot{1,Feature(FID,3)},K,extrinsic);
                [PVector.Feature(3*FID-2,1),PVector.Feature(3*FID-1,1)] = FuncV2PT(V1);
            end;
        end;
    
        Feature(FID,3*Feature(FID,5)+6) = PoseID-(start_ImageNum-1);
        Feature(FID,3*Feature(FID,5)+7:3*Feature(FID,5)+8) = Image(i+1,2:3);
        Feature(FID,5) = Feature(FID,5)+1;
%         clear V1 V2 Max ID Omegai;
        continue;
    
    end;
end;