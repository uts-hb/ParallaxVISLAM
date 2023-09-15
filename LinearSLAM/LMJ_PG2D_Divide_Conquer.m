function [GMap,LM_L] = LMJ_PG2D_Divide_Conquer(LM,LocalMapNum)

L = 1;
LM_L{L} = LM;
clear LM;

for i=1:LocalMapNum;
    LM_L{L}{i}.sp = LM_L{L}{i}.Ref;
    LM_L{L}{i} = Funcwrapst(LM_L{L}{i});
end;

while LocalMapNum>1;
    N2 = rem(LocalMapNum,2);
    LocalMapNum = ceil(LocalMapNum/2);
    
    for i=1:LocalMapNum;
        
        if i<LocalMapNum;
            NumLM = 2;
        elseif i==LocalMapNum;
            if N2;
                NumLM = 1;
            else
                NumLM = 2;
            end;
        end;
        
        for j=1:NumLM;                        
            fprintf('Join Level %d Local Map %d\n', L, 2*(i-1)+j);    
            if j==1;
                GMap = LM_L{L}{2*(i-1)+j};
            else
                [GMap_End] = FuncTransform_PG2D(GMap,LM_L{L}{2*(i-1)+j}.Ref);
                [GMap_End] = Funcwrapst(GMap_End);

                [GMap] = FuncLinearLS_PG2D(GMap_End,LM_L{L}{2*(i-1)+j});
            end;
                                    
        end;
        
        [GMap] = Funcwrapst(GMap);
        
        if rem(i,2) == 0;
            if GMap.Ref>GMap.sp;
                [GMap] = FuncTransform_PG2D(GMap,GMap.sp);
            end;
        end;
        
        [GMap] = Funcwrapst(GMap);
        
        LM_L{L+1}{i} = GMap;        
        fprintf('Generate Level %d Local Map %d\n\n', L+1, i);
        
    end;
    
    L = L+1;
    
end;

%%
m = min(GMap.st(:,1));
if GMap.Ref>m;
    [GMap] = FuncTransform_PG2D(GMap,m);
end;
   
