function [Feature,PVector,xVector,feature_id_change] = rearrange_featureID(Feature,PVector,xVector)



% if rem(size(PVector.Feature,1),3) ==2
%     PVector.Feature(end+1,1) = 0;
% end 

zero_id = [];
for i = 1 : size(Feature,1)
    if(max(zeros(2,1)==PVector.Feature(3*(i-1)+1:3*(i-1)+2,1)) == 1)
        zero_id = [zero_id;(3*(i-1)+1:3*(i-1)+3)'];
    end
end

PVector.Feature(zero_id) = []; 
Feature( all(~Feature,2), : ) = [];


feature_id_change = []; 

for i = 1 : size(Feature,1)
    if i ~= Feature(i,1)
        mask_to_change = (xVector.FID == Feature(i,1));
        xVector.FID(mask_to_change) = i;
        feature_id_change = [feature_id_change;Feature(i,1),i];
        Feature(i,1) = i;
    else 
        feature_id_change = [feature_id_change;Feature(i,1),i];
    end
end