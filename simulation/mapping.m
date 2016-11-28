t0 = [1 1; 3 0.5; 3.5 1.5; ]
t1 = [1.2 1.2; 3.2 0.7;4 4]
c = [2 3.25]
D = pdist2(t0, t1)

[~,I] = min(D, [],2)
temp = D
temp([1; 2; 3; 4], [1; 2; 2; 3;]) = 1

[Y,I] = min(D, [], 2);
B = NaN(size(D));
B(sub2ind(size(D), 1:length(I), I')) = Y

h = figure(1);
clf;
hold on;

b = plot(t0(:,1), t0(:,2), 'r.', 'MarkerSize', 5);
b = plot(t1(:,1), t1(:,2), 'b+', 'MarkerSize', 5);
b = plot(c(1), c(2), 'kd', 'MarkerSize', 8);


xlim([0 5]);
ylim([0 5]);
zlim([-10 10]);