function [GMap_End] = FuncTransform_PF2D(GMap,Ref)

%%
if GMap.Ref == Ref;
    GMap_End = GMap;
else
%%    
    a = find(GMap.st(:,1)==-Ref);
    nn = a(end);
    
    t = GMap.st(nn-2:nn-1,2);
    phi = GMap.st(nn,2);
    R = FuncRMatrix2D(phi);

    n = length(GMap.st(:,1));
    
    GMap_End.st = zeros(n,2);
    GMap_End.st(:,1) = GMap.st(:,1);
    GMap_End.st(a,1) = -GMap.Ref;
    GMap_End.Ref = Ref;
    
    i = 1;
    while i<=n;
        if GMap.st(i,1) <= 0;
            if i == nn-2;
                GMap_End.st(i:i+1,2) = -R*t;
                GMap_End.st(i+2,2) = -phi;
            else
                GMap_End.st(i:i+1,2) = R*(GMap.st(i:i+1,2)-t);
                GMap_End.st(i+2,2) = GMap.st(i+2,2)-phi;
            end;
            i = i+3;
        else
            GMap_End.st(i:i+1,2) = R*(GMap.st(i:i+1,2)-t);
            i = i+2;
        end;
    end;

%%
    t = GMap_End.st(nn-2:nn-1,2);
    phi = GMap_End.st(nn,2);

    R = FuncRMatrix2D(phi);
    dR = FuncdR2D(phi);

    ID1 = [];
    ID2 = [];
    Val = [];

    i = 1;
    while i<=n;
        if GMap_End.st(i,1) <= 0;
            if i == nn-2;
                ID1 = [ID1;i;i+1;i;i+1;i;i+1;i+2];
                ID2 = [ID2;nn-2;nn-2;nn-1;nn-1;nn;nn;nn];
                Val = [Val;-R(:,1);-R(:,2);-dR*t;-1];
            else                
                ID1 = [ID1;i;i+1;i;i+1;i;i+1;i;i+1;i;i+1;i+2;i+2];
                ID2 = [ID2;nn-2;nn-2;nn-1;nn-1;nn;nn;i;i;i+1;i+1;nn;i+2];
                Val = [Val;-R(:,1);-R(:,2);dR*(GMap_End.st(i:i+1,2)-t);R(:,1);R(:,2);-1;1];               
            end;
            i = i+3;
        else
            ID1 = [ID1;i;i+1;i;i+1;i;i+1;i;i+1;i;i+1];
            ID2 = [ID2;nn-2;nn-2;nn-1;nn-1;nn;nn;i;i;i+1;i+1];
            Val = [Val;-R(:,1);-R(:,2);dR*(GMap_End.st(i:i+1,2)-t);R(:,1);R(:,2)];
            i = i+2;
        end;
        
    end;

    J = sparse(ID1,ID2,Val);
    GMap_End.I = J'*GMap.I*J;

end;
    
