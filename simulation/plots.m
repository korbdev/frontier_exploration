%avg_vec
%step_vec
%journey_vec

NN_17 = load('map_17/data_NN.mat');
FTA_17 = load('map_17/data_FTA.mat');

NN_174 = load('map_174/data_NN.mat');
FTA_174 = load('map_174/data_FTA.mat');

NN_19 = load('map_19/data_NN.mat');
FTA_19 = load('map_19/data_FTA.mat');

NN_30 = load('map_30/data_NN.mat');
FTA_30 = load('map_30/data_FTA.mat');


X_17_NN = 0:1:size(NN_17.step_vec,2)-1;
X_17_FTA = 0:1:size(FTA_17.step_vec,2)-1;

X_174_NN = 0:1:size(NN_174.step_vec,2)-1;
X_174_FTA = 0:1:size(FTA_174.step_vec,2)-1;

X_19_NN = 0:1:size(NN_19.step_vec,2)-1;
X_19_FTA = 0:1:size(FTA_19.step_vec,2)-1;

X_30_NN = 0:1:size(NN_30.step_vec,2)-1;
X_30_FTA = 0:1:size(FTA_30.step_vec,2)-1;

X_NN = [size(NN_30.step_vec,2) size(NN_19.step_vec,2) size(NN_17.step_vec,2) size(NN_174.step_vec,2)];
X_FTA = [size(FTA_30.step_vec,2) size(FTA_19.step_vec,2) size(FTA_17.step_vec,2) size(FTA_174.step_vec,2)];

clf
fig1 = figure(1);
hold on;

plot(X_30_NN, NN_30.step_vec, 'b-*')
plot(X_30_FTA, FTA_30.step_vec, 'r-o')

plot(X_30_NN, NN_30.avg_vec, 'c-*')
plot(X_30_FTA, FTA_30.avg_vec, 'y-o')
legend('NN step','FTA step','NN avg','FTA avg','Location','northwest','Orientation','horizontal');
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

saveas(fig1, '~/research/frontier_exploration/paper/figures/fig41.png', 'png');

fig2 = figure(2);
hold on;

plot(X_19_NN, NN_19.step_vec, 'b-*')
plot(X_19_FTA, FTA_19.step_vec, 'r-o')

plot(X_19_NN, NN_19.avg_vec, 'c-*')
plot(X_19_FTA, FTA_19.avg_vec, 'y-o')
legend('NN step','FTA step','NN avg','FTA avg','Location','northwest','Orientation','horizontal');
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

saveas(fig2, '~/research/frontier_exploration/paper/figures/fig42.png', 'png');

fig3 = figure(3);
hold on;

plot(X_17_NN, NN_17.step_vec, 'b-*')
plot(X_17_FTA, FTA_17.step_vec, 'r-o')

plot(X_17_NN, NN_17.avg_vec, 'c-*')
plot(X_17_FTA, FTA_17.avg_vec, 'y-o')
legend('NN step','FTA step','NN avg','FTA avg','Location','northwest','Orientation','horizontal');
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

saveas(fig3, '~/research/frontier_exploration/paper/figures/fig43.png', 'png');

fig4 = figure(4);
hold on;

plot(X_174_NN, NN_174.step_vec, 'b-*')
plot(X_174_FTA, FTA_174.step_vec, 'r-o')

plot(X_174_NN, NN_174.avg_vec, 'c-*')
plot(X_174_FTA, FTA_174.avg_vec, 'y-o')

legend('NN step','FTA step','NN avg','FTA avg','Location','northwest','Orientation','horizontal');

ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

saveas(fig4, '~/research/frontier_exploration/paper/figures/fig44.png', 'png');

%plot(X_174_NN, NN_174.step_vec, 'b-*')
%plot(X_174_FTA, FTA_174.step_vec, 'r-o')

%plot(X_19_NN, NN_19.step_vec, 'b-*')
%plot(X_19_FTA, FTA_19.step_vec, 'r-o')

xlim([0 66]);
%ylim([0 400]);

%Tracel distance

figure(5);
hold on;

plot(X_17_NN, NN_17.journey_vec, 'b:*', 'MarkerSize', 8)
plot(X_17_FTA, FTA_17.journey_vec, 'r-*', 'MarkerSize', 8)

plot(X_174_NN, NN_174.journey_vec, 'b:x', 'MarkerSize', 8)
plot(X_174_FTA, FTA_174.journey_vec, 'r-x', 'MarkerSize', 8)

plot(X_19_NN, NN_19.journey_vec, 'b:d', 'MarkerSize', 8)
plot(X_19_FTA, FTA_19.journey_vec, 'r-d', 'MarkerSize', 8)

plot(X_30_NN, NN_30.journey_vec, 'b:d', 'MarkerSize', 8)
plot(X_30_FTA, FTA_30.journey_vec, 'r-d', 'MarkerSize', 8)

journeys_NN = [sum(NN_30.step_vec),sum(NN_19.step_vec),sum(NN_17.step_vec),sum(NN_174.step_vec)]
journeys_FTA = [sum(FTA_30.step_vec),sum(FTA_19.step_vec),sum(FTA_17.step_vec),sum(FTA_174.step_vec)]
%journeys_FTA = [FTA_17.journey_vec, FTA_174.journey_vec, FTA_19.journey_vec, FTA_30.journey_vec]

avg_NN = [NN_30.avg_vec(X_NN(1)) NN_19.avg_vec(X_NN(2)) NN_17.avg_vec(X_NN(3)) NN_174.avg_vec(X_NN(4))]
avg_FTA = [FTA_30.avg_vec(X_FTA(1)) FTA_19.avg_vec(X_FTA(2)) FTA_17.avg_vec(X_FTA(3)) FTA_174.avg_vec(X_FTA(4))]

data = [journeys_NN' journeys_FTA']
percent = (data(:,2) - data(:,1))./data(:,1)

percent_avg = (avg_FTA - avg_NN) ./ avg_NN

percent_step = (X_FTA-X_NN)./X_NN

figure(6);

labels = {'A', 'B', 'C', 'D'};

h=bar(data);
h(1).FaceColor = [0.75 0.75 0.75];
h(2).FaceColor = [0.25 0.25 0.25];
set(gca, 'XTickLabel',labels, 'XTick',1:numel(labels));
legend('NN', 'FTA');