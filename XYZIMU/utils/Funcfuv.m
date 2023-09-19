
function [u,v] = Funcfuv(Pi,Xj,K)

% Alphai = Pi(1);   
% Betai = Pi(2);
% Gammai = Pi(3);
% Txi = Pi(4);
% Tyi = Pi(5);
% Tzi = Pi(6);
% Xxj = Xj(1);
% Xyj = Xj(2);
% Xzj = Xj(3);
% 
% % u,v
% a1 = (cos(Alphai)*cos(Betai))*(Xxj-Txi)+(sin(Alphai)*cos(Betai))*(Xyj-Tyi)-sin(Betai)*(Xzj-Tzi);
% a2 = (cos(Alphai)*sin(Betai)*sin(Gammai)-sin(Alphai)*cos(Gammai))*(Xxj-Txi)+(sin(Alphai)*sin(Betai)*sin(Gammai)+cos(Alphai)*cos(Gammai))*(Xyj-Tyi)+cos(Betai)*sin(Gammai)*(Xzj-Tzi);
% b = (cos(Alphai)*sin(Betai)*cos(Gammai)+sin(Alphai)*sin(Gammai))*(Xxj-Txi)+(sin(Alphai)*sin(Betai)*cos(Gammai)-cos(Alphai)*sin(Gammai))*(Xyj-Tyi)+(cos(Betai)*cos(Gammai))*(Xzj-Tzi);
% aa = K*[a1;a2;b];
% u = aa(1)/aa(3);
% v = aa(2)/aa(3);


% For rotaion so(3)
% Reprojection equation can be written as follows
% aa = K*R'*(Xj(1:3)'-Pi(4:6)') where R = eul2rotm(Pi(1:3))
% 
R = Pi.R;
t = Pi.p;
aa = K*R'*(Xj(1:3)'-t');

u = aa(1)/aa(3);
v = aa(2)/aa(3);

