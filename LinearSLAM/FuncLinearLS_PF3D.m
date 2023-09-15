function [GMap] = FuncLinearLS_PF3D(LM1,LM2)

n1 = length(LM1.st(:,1));
n2 = length(LM2.st(:,1));

b = [LM1.st(:,2);LM2.st(:,2)];

I = sparse(n1+n2,n1+n2);
I(1:n1,1:n1) = LM1.I;
I(n1+1:n1+n2,n1+1:n1+n2) = LM2.I;

Val = [];
ID1 = (1:1:n1)';
ID2 = (1:1:n1)';
Val(1:n1,1) = 1;
XGID = LM1.st(:,1);
Xn = n1;

i = 1;
while i<=n2;
    if LM2.st(i,1)<=0;
        Pn = LM2.st(i,1);
        XGID = [XGID;Pn;Pn;Pn;Pn;Pn;Pn];        
        ID1 = [ID1;(n1+i:n1+i+5)'];
        ID2 = [ID2;(Xn+1:Xn+6)'];
        Val = [Val;1;1;1;1;1;1];
        Xn = Xn+6;
        i = i+6;
    else
        Fn = LM2.st(i,1);
        a = find(XGID==Fn);
        if a;
            ID1 = [ID1;n1+i;n1+i+1;n1+i+2];
            ID2 = [ID2;a];
            Val = [Val;1;1;1];
        else
            XGID = [XGID;Fn;Fn;Fn];        
            ID1 = [ID1;n1+i;n1+i+1;n1+i+2];
            ID2 = [ID2;Xn+1;Xn+2;Xn+3];
            Val = [Val;1;1;1];
            Xn = Xn+3;
        end;
        i = i+3;
    end;
end;
    
A = sparse(ID1,ID2,Val);

XG = (A'*I*A)\(A'*I*b);

GMap.I = A'*I*A;
GMap.st = [XGID,XG];
GMap.Ref = LM2.Ref;

