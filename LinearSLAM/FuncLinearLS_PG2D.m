function [GMap] = FuncLinearLS_PG2D(LM1,LM2)

n1 = length(LM1.st(:,1));
n2 = length(LM2.st(:,1));

b = [LM1.st(:,2);LM2.st(:,2)];

I = sparse(n1+n2,n1+n2);
I(1:n1,1:n1) = LM1.I;
I(n1+1:n1+n2,n1+1:n1+n2) = LM2.I;


ID1 = (1:1:n1)';
ID2 = (1:1:n1)';
Val(1:n1,1) = 1;
XGID = LM1.st(:,1);
Xn = n1;

i = 1;
while i<=n2;
    Pn = LM2.st(i,1);
    a = find(XGID==Pn);
    if a;
        ID1 = [ID1;n1+i;n1+i+1;n1+i+2];
        ID2 = [ID2;a];
        Val = [Val;1;1;1];
        Err = b(n1+i+2,1)-b(a(end),1);
        if Err>pi;
            b(n1+i+2,1) = b(n1+i+2,1)-2*pi;
        elseif Err<-pi;
            b(n1+i+2,1) = b(n1+i+2,1)+2*pi;
        end;
    else        
        XGID = [XGID;Pn;Pn;Pn];        
        ID1 = [ID1;n1+i;n1+i+1;n1+i+2];
        ID2 = [ID2;Xn+1;Xn+2;Xn+3];
        Val = [Val;1;1;1];
        Xn = Xn+3;
    end;
    i = i+3;
end;
    
A = sparse(ID1,ID2,Val);

XG = (A'*I*A)\(A'*I*b);

GMap.I = A'*I*A;
GMap.st = [XGID,XG];
GMap.Ref = LM2.Ref;
GMap.sp = LM1.sp;
