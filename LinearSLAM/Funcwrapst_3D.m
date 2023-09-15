function [GMap] = Funcwrapst_3D(GMap)

for i=3:6:length(GMap.st);
    for j=1:3;
        while GMap.st(i+j,2)>pi || GMap.st(i+j,2)<-pi;
            GMap.st(i+j,2) = wrap(GMap.st(i+j,2));
        end;
    end;
end;