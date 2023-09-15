function [Pose,Feature] = FuncPlotFigure_PF2D(GMap)

X = GMap.st;

Pose = [];
Feature = [];
FID = [];
PID = [];

n = length(X(:,1));
i = 1;
while i<=n;
    if X(i,1)<=0;
        Pose = [Pose;X(i:i+2,2)'];
        PID = [PID;-X(i,1)];
        i = i+3;
    else
        Feature = [Feature;X(i:i+1,2)'];
        FID = [FID;X(i,1)];
        i = i+2;
    end;
end;

%%
[PB,PIX] = sort(PID);
Pose = Pose(PIX,:);

[FB,FIX] = sort(FID);
Feature = Feature(FIX,:);

%%
hold on;
plot(Pose(:,1),Pose(:,2),'r.');
plot(Feature(:,1),Feature(:,2),'k.');
axis equal;

%%
Pose = [PB,Pose];
Feature = [FB,Feature];


