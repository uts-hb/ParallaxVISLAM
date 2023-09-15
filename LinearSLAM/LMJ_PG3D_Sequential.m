function [GMap] = LMJ_PG3D_Sequential(LM,LocalMapNum)

for i=1:LocalMapNum;
    
    fprintf('Join Local Map %d\n', i);
    
    LM{i}.sp = LM{i}.Ref;
    LM{i} = Funcwrapst_3D(LM{i});
	if i==1;
        GMap = LM{i};
	else
        [GMap_End] = FuncTransform_PG3D(GMap,LM{i}.Ref);
        [GMap] = FuncLinearLS_PG3D(GMap_End,LM{i});
        [GMap] = Funcwrapst_3D(GMap);
        
	end;
end;

%%
m = min(GMap.st(:,1));
if GMap.Ref>m;
    [GMap] = FuncTransform_PG3D(GMap,m);
end;
   
