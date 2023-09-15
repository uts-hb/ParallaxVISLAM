function [GMap,LM_L] = LMJ_PF2D_Divide_Conquer(LM,LocalMapNum)

L = 1;
LM_L{L} = LM;
clear LM;

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
                [GMap_End] = FuncTransform_PF2D(GMap,LM_L{L}{2*(i-1)+j}.Ref);
                [GMap] = FuncLinearLS_PF2D(GMap_End,LM_L{L}{2*(i-1)+j});        
            end;
        end;
        
        if rem(i,2) == 0;
            a = find(GMap.st(:,1)<=0);
            m = min(-GMap.st(a(:),1));
%             m = min(GMap.PID);
            if GMap.Ref>m;
                [GMap] = FuncTransform_PF2D(GMap,m);
            end;
        end;
        
        LM_L{L+1}{i} = GMap;        
        fprintf('Generate Level %d Local Map %d\n\n', L+1, i);
        
    end;
    
    L = L+1;
    
end;

%%
a = find(GMap.st(:,1)<=0);
m = min(-GMap.st(a(:),1));
if GMap.Ref>m;
    [GMap] = FuncTransform_PF2D(GMap,m);
end; 
   
