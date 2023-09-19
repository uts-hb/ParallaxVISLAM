function [Feature,PVector,xVector,feature_id_change] = delete_feature_one(Feature,PVector,xVector)


%% Delete features observed only once
mask_1_ = (Feature(:,5)==1);
mask_1 = find(mask_1_);
feat_id_only_once = Feature(mask_1,1);
for i = 1 : length(feat_id_only_once)
    PVector.Feature(3*(feat_id_only_once(i)-1)+1:3*(feat_id_only_once(i)-1)+2) = 0;
    mask_fid = (xVector.FID == feat_id_only_once(i));
    xVector.FID(mask_fid) = 0;
    xVector.PID(mask_fid) = 0;

    fid = find(mask_fid);
    for i = 1 : length(fid)
        xVector.u(2*(fid(i)-1)+1 : 2*(fid(i)-1)+2) = 0;
    end
end
PVector.Feature( all(~PVector.Feature,2), : ) = [];
xVector.u( all(~xVector.u,2), : ) = [];
xVector.PID( all(~xVector.PID,2), : ) = [];
xVector.FID( all(~ xVector.FID,2), : ) = [];
Feature = Feature(~mask_1_,:);

%% Delete all 0 features between feature ID
% max_feat_id = max(Feature(:,1));
% 
% if max_feat_id <= size(Feature,1)
%     Feature = Feature(1:max_feat_id,:);
% elseif max_feat_id > size(Feature,1)
%     Feature = Feature(1:size(Feature,1),:);
% end

% mask_no_feature = (Feature(:,1)==0);
% no_feature = find(mask_no_feature);
% if ~isempty(no_feature)
%     for i = 1 : size(no_feature,1)
%         PVector.Feature(3*(no_feature(i,1)-1)+1:3*(no_feature(i,1)-1)+3,1) = -1;
%     end
% end
% mask_no_feature_= (PVector.Feature(:,1)==-1);
% if ~isempty(mask_no_feature_)
%     PVector.Feature = PVector.Feature(~mask_no_feature_);
% end

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
