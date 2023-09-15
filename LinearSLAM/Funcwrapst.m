function [GMap] = Funcwrapst(GMap)

for i=3:3:length(GMap.st);
    while GMap.st(i,2)>pi || GMap.st(i,2)<-pi;
        GMap.st(i,2) = wrap(GMap.st(i,2));
    end;
end;