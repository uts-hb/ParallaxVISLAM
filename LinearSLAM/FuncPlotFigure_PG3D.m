function [Pose] = FuncPlotFigure_PG3D(GMap)

X = GMap.st;

Pose = [];
PID = [];

n = length(X(:,1));
i = 1;
while i<=n;
    PID = [PID;X(i,1)];
	Pose = [Pose;X(i:i+5,2)'];
	i = i+6;
end;

%%
[B,IX] = sort(PID);
Pose = Pose(IX,:);

%%
hold on;
plot3(Pose(:,1),Pose(:,2),Pose(:,3),'r.-');
axis equal;

Pose = [B,Pose];

