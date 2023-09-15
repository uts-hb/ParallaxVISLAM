function [Pose] = FuncPlotFigure_PG2D(GMap)

X = GMap.st;

Pose = [];
PID = [];

n = length(X(:,1));
i = 1;
while i<=n;
    PID = [PID;X(i,1)];
	Pose = [Pose;X(i:i+2,2)'];
	i = i+3;
end;

%%
[B,IX] = sort(PID);
Pose = Pose(IX,:);

%%
hold on;
plot(Pose(:,1),Pose(:,2),'r.-');
axis equal;

Pose = [B,Pose];

