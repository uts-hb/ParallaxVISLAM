
function [PVector] = FuncUpdate(PVector,DeltaP,DeltaF,FixVa)

for i = 1 : size(PVector.Pos,2)

    R = PVector.Rot{i};
    delta_phi = DeltaP(6*(i-1)+1:6*(i-1)+3,1);
    p = PVector.Pos(:,i);
    delta_p = DeltaP(6*(i-1)+4:6*(i-1)+6,1);

    PVector.Rot{i} = R * SO3.exp(delta_phi); 
    PVector.Rot{i}  = renormalizeRotMat(PVector.Rot{i});

        PVector.Pos(:,i) = p + R*delta_p; 

        if i == 2
            delP = R*delta_p;
            if FixVa==3;%% Fix Z
                delP(3) = 0;
            elseif FixVa==2;%% Fix Y
                delP(2) = 0;
            elseif FixVa==1;%% Fix X
                delP(1) = 0;
            end
            PVector.Pos(:,i) = p + delP;
        else
            PVector.Pos(:,i) = p + R*delta_p;
        end
end 


PVector.Feature = PVector.Feature+DeltaF;


