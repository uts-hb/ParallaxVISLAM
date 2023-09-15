function [Pose,Feature] = FuncPlotFigure_PF3D(GMap)

X = GMap.st;

Pose = [];
Feature = [];
FID = [];
PID = [];


n = length(X(:,1));
i = 1;
while i<=n;
    if X(i,1)<=0;
        Pose = [Pose;X(i:i+5,2)'];
        PID = [PID;-X(i,1)];
        i = i+6;
    else
        Feature = [Feature;X(i:i+2,2)'];
        FID = [FID;X(i,1)];
        i = i+3;
    end;
end;

%%
[PB,PIX] = sort(PID);
Pose = Pose(PIX,:);

[FB,FIX] = sort(FID);
Feature = Feature(FIX,:);

%%
hold on;

plot3(Pose(:,1),Pose(:,2),Pose(:,3),'r.');
plot3(Feature(:,1),Feature(:,2),Feature(:,3),'k.');
axis equal;

%%
Pose = [PB,Pose];
Feature = [FB,Feature];

