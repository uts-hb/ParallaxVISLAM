
% function [duda,dvda,dudb,dvdb,dudt,dvdt,dudTx,dvdTx,dudTy,dvdTy,dudTz,dvdTz,dudXx,dvdXx,dudXy,dvdXy,dudXz,dvdXz] = FuncduvdP(Alphai,Betai,Gammai,Txi,Tyi,Tzi,Xxj,Xyj,Xzj)
function [dudP,dvdP] = FuncduvdP(Pi,Xj,K)


%u,v
% a1 = (cos(Alphai)*cos(Betai))*(Xxj-Txi)+(sin(Alphai)*cos(Betai))*(Xyj-Tyi)-sin(Betai)*(Xzj-Tzi);
% a2 = (cos(Alphai)*sin(Betai)*sin(Gammai)-sin(Alphai)*cos(Gammai))*(Xxj-Txi)+(sin(Alphai)*sin(Betai)*sin(Gammai)+cos(Alphai)*cos(Gammai))*(Xyj-Tyi)+cos(Betai)*sin(Gammai)*(Xzj-Tzi);
% b = (cos(Alphai)*sin(Betai)*cos(Gammai)+sin(Alphai)*sin(Gammai))*(Xxj-Txi)+(sin(Alphai)*sin(Betai)*cos(Gammai)-cos(Alphai)*sin(Gammai))*(Xyj-Tyi)+(cos(Betai)*cos(Gammai))*(Xzj-Tzi);
% a = K*[a1;a2;b];

R = Pi.R;
t = Pi.p';
x = K*R'*(Xj(1:3)'-t);

dudx = [1/x(3),0,-x(1)/x(3)^2;
                0,1/x(3),-x(2)/x(3)^2;];
dxdRi = K*SO3.skew(R'*(Xj(1:3)'-t));
dudRi = dudx * dxdRi;

dxdPT = -K*R';
dudPT = dudx*dxdPT; 

dxdXj = K*R';
dudXj = dudx*dxdXj;

dudX = [dudRi, dudPT, dudXj]; 
dudP = dudX(1,:);
dvdP = dudX(2,:); 




% %duda,dvda
% da1da = -sin(Alphai)*cos(Betai)*(Xxj-Txi)+cos(Alphai)*cos(Betai)*(Xyj-Tyi);
% da2da = (-sin(Alphai)*sin(Betai)*sin(Gammai)-cos(Alphai)*cos(Gammai))*(Xxj-Txi)+(cos(Alphai)*sin(Betai)*sin(Gammai)-sin(Alphai)*cos(Gammai))*(Xyj-Tyi);
% dbda = (-sin(Alphai)*sin(Betai)*cos(Gammai)+cos(Alphai)*sin(Gammai))*(Xxj-Txi)+(cos(Alphai)*sin(Betai)*cos(Gammai)+sin(Alphai)*sin(Gammai))*(Xyj-Tyi);
% dada = K*[da1da;da2da;dbda];
% duda = (dada(1)*a(3)-a(1)*dada(3))/a(3)^2;
% dvda = (dada(2)*a(3)-a(2)*dada(3))/a(3)^2;
% 
% %dudb,dvdb
% da1db = -cos(Alphai)*sin(Betai)*(Xxj-Txi)-sin(Alphai)*sin(Betai)*(Xyj-Tyi)-cos(Betai)*(Xzj-Tzi);
% da2db = cos(Alphai)*cos(Betai)*sin(Gammai)*(Xxj-Txi)+sin(Alphai)*cos(Betai)*sin(Gammai)*(Xyj-Tyi)-sin(Betai)*sin(Gammai)*(Xzj-Tzi);
% dbdb =  cos(Alphai)*cos(Betai)*cos(Gammai)*(Xxj-Txi)+sin(Alphai)*cos(Betai)*cos(Gammai)*(Xyj-Tyi)-sin(Betai)*cos(Gammai)*(Xzj-Tzi);
% dadb = K*[da1db;da2db;dbdb];
% dudb = (dadb(1)*a(3)-a(1)*dadb(3))/a(3)^2;
% dvdb = (dadb(2)*a(3)-a(2)*dadb(3))/a(3)^2; 
% % dudb = (da1db*b-a1*dbdb)/b^2;
% % dvdb = (da2db*b-a2*dbdb)/b^2;
% 
% %dudg,dvdg
% da1dg = 0;
% da2dg = (cos(Alphai)*sin(Betai)*cos(Gammai)+sin(Alphai)*sin(Gammai))*(Xxj-Txi)+(sin(Alphai)*sin(Betai)*cos(Gammai)-cos(Alphai)*sin(Gammai))*(Xyj-Tyi)+cos(Betai)*cos(Gammai)*(Xzj-Tzi);
% dbdg = (-cos(Alphai)*sin(Betai)*sin(Gammai)+sin(Alphai)*cos(Gammai))*(Xxj-Txi)+(-sin(Alphai)*sin(Betai)*sin(Gammai)-cos(Alphai)*cos(Gammai))*(Xyj-Tyi)-cos(Betai)*sin(Gammai)*(Xzj-Tzi);
% dadg = K*[da1dg;da2dg;dbdg];
% dudg = (dadg(1)*a(3)-a(1)*dadg(3))/a(3)^2;
% dvdg = (dadg(2)*a(3)-a(2)*dadg(3))/a(3)^2;
% % dudt = (da1dt*b-a1*dbdt)/b^2;
% % dvdt = (da2dt*b-a2*dbdt)/b^2;
% 
% %dudXx,dvdXx
% da1dXx = cos(Alphai)*cos(Betai);
% da2dXx = cos(Alphai)*sin(Betai)*sin(Gammai)-sin(Alphai)*cos(Gammai);
% dbdXx = cos(Alphai)*sin(Betai)*cos(Gammai)+sin(Alphai)*sin(Gammai);
% dadXx = K*[da1dXx;da2dXx;dbdXx];
% dudXx = (dadXx(1)*a(3)-a(1)*dadXx(3))/a(3)^2;
% dvdXx = (dadXx(2)*a(3)-a(2)*dadXx(3))/a(3)^2;
% % dudXx = (da1dXx*b-a1*dbdXx)/b^2;
% % dvdXx = (da2dXx*b-a2*dbdXx)/b^2;
% 
% %dudXy,dvdXy
% da1dXy = sin(Alphai)*cos(Betai);
% da2dXy = sin(Alphai)*sin(Betai)*sin(Gammai)+cos(Alphai)*cos(Gammai);
% dbdXy = sin(Alphai)*sin(Betai)*cos(Gammai)-cos(Alphai)*sin(Gammai);
% dadXy = K*[da1dXy;da2dXy;dbdXy];
% dudXy = (dadXy(1)*a(3)-a(1)*dadXy(3))/a(3)^2;
% dvdXy = (dadXy(2)*a(3)-a(2)*dadXy(3))/a(3)^2;
% % dudXy = (da1dXy*b-a1*dbdXy)/b^2;
% % dvdXy = (da2dXy*b-a2*dbdXy)/b^2;
% 
% %dudXz,dvdXz
% da1dXz = -sin(Betai);
% da2dXz = cos(Betai)*sin(Gammai);
% dbdXz = cos(Betai)*cos(Gammai);
% dadXz = K*[da1dXz;da2dXz;dbdXz];
% dudXz = (dadXz(1)*a(3)-a(1)*dadXz(3))/a(3)^2;
% dvdXz = (dadXz(2)*a(3)-a(2)*dadXz(3))/a(3)^2;
% % dudXz = (da1dXz*b-a1*dbdXz)/b^2;
% % dvdXz = (da2dXz*b-a2*dbdXz)/b^2;
% 
% %dudT,dvdT
% dudTx = -dudXx;
% dvdTx = -dvdXx;
% dudTy = -dudXy;
% dvdTy = -dvdXy;
% dudTz = -dudXz;
% dvdTz = -dvdXz;
% 
% dudP = [duda,dudb,dudg,dudTx,dudTy,dudTz,dudXx,dudXy,dudXz];
% dvdP = [dvda,dvdb,dvdg,dvdTx,dvdTy,dvdTz,dvdXx,dvdXy,dvdXz];
