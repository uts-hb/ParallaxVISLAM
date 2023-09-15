function dRid = FuncdRi(dRi,Ri)

F1 = Ri(1,2)/Ri(1,1);
F3 = Ri(2,3)/Ri(3,3);
F5 = Ri(1,1)^2+Ri(1,2)^2;
F4 = sqrt(F5);
F2 = -Ri(1,3)/F4;

% AAAAA = cos(atan(F2));
% if abs(AAAAA)<0.1;
%     AAAAA
% end;

dAdF1 = Funcdatan(F1);
dBdF2 = Funcdatan(F2);
dGdF3 = Funcdatan(F3);

dF1d = (dRi(1,2)*Ri(1,1)-Ri(1,2)*dRi(1,1))/Ri(1,1)^2;
dF3d = (dRi(2,3)*Ri(3,3)-Ri(2,3)*dRi(3,3))/Ri(3,3)^2;

dF4dF5 = FuncdDikdD2(F5);
dF5d = 2*Ri(1,1)*dRi(1,1)+2*Ri(1,2)*dRi(1,2);
dF4d = dF4dF5*dF5d;

dF2d = (-dRi(1,3)*F4+Ri(1,3)*dF4d)/F5;

dRid = [dAdF1*dF1d;dBdF2*dF2d;dGdF3*dF3d];