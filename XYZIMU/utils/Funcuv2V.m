function V = Funcuv2V(uv,R,K,extrinsic)

RMatrix = R*extrinsic(1:3,1:3);
x = [uv(1);uv(2);1];
V = K*RMatrix'\x;
