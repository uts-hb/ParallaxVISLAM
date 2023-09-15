function [GMap] = LMJ_PG2D_Sequential(LM,LocalMapNum)

for i=1:LocalMapNum;
    
    fprintf('Join Local Map %d\n', i);
    
    LM{i}.sp = LM{i}.Ref;
    LM{i} = Funcwrapst(LM{i});
	if i==1;
        GMap = LM{i};
	else
        [GMap_End] = FuncTransform_PG2D(GMap,LM{i}.Ref);
        [GMap_End] = Funcwrapst(GMap_End);
        
        [GMap] = FuncLinearLS_PG2D(GMap_End,LM{i});
        [GMap] = Funcwrapst(GMap);        
	end;
end;

%%
m = min(GMap.st(:,1));
if GMap.Ref>m;
    [GMap] = FuncTransform_PG2D(GMap,m);
end;
   
