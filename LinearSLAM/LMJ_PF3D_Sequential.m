function [GMap] = LMJ_PF3D_Sequential(LM,LocalMapNum)

for i=1:LocalMapNum;
    
    fprintf('Join Local Map %d\n', i);
    
	if i==1;
        GMap = LM{i};
	else
        [GMap_End] = FuncTransform_PF3D(GMap,LM{i}.Ref);
        [GMap] = FuncLinearLS_PF3D(GMap_End,LM{i});
	end;
end;

%%
a = find(GMap.st(:,1)<=0);
m = min(-GMap.st(a(:),1));

if GMap.Ref>m;
    [GMap] = FuncTransform_PF3D(GMap,m);
end;
   
