points_1 = [1 2;3 4;5 6;]
points_2 = [7 8;10 11;11 12;]
points_3 = [4 8;5 8;6 8;]

points_4 = [1 2;3 4;5 6;]
points_5 = [8 8;10 11;11 12;]
points_6 = [4 8;5 8;6 8;]

f_1 = Frontier(points_1, 0),f_2 = Frontier(points_2, 0),f_3 = Frontier(points_3, 0);
f_4 = Frontier(points_4, 0),f_5 = Frontier(points_5, 0),f_6 = Frontier(points_6, 0);

frontiers_1 = [f_1;f_2;f_3];
frontiers_2 = [f_4;f_5;f_6];

f = FrontierGraph(frontiers_1)
g = FrontierGraph(frontiers_2)

f.updateGraph(frontiers_2)

%m = ones(4,4)

%f.removeNodes([4], m)