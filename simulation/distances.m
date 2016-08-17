%Y_2 = [2 2; 2 3; 2 4; 2 5; 2 6; 2 7; 2 8;
%    2 8; 3 8; 4 8; 5 8; 6 8; 7 8; 8 8;
%    8 2; 8 3; 8 4; 8 5; 8 6; 8 7; 8 8;]

%Y_2 = [13 2; 13 3; 13 4; 13 5; 13 6; 13 7; 13 8;
%    13 8; 14 8; 15 8; 16 8; 17 8; 18 8; 19 8;
%    19 2; 19 3; 19 4; 19 5; 19 6; 19 7; 19 8;]
%Y_2 = [0 0; 0 0; 0 0; 0 0; 0 0; 0 0; 0 0;
%    2 8; 3 8; 4 8; 5 8; 6 8; 7 8; 8 8;
%    0 0; 0 0; 0 0; 0 0; 0 0; 0 0; 0 0;]


%Y = [2 7;2 8; 3 8; 4 8; 5 8; 6 8; 7 8; 8 8;]

Y = [3 4; 3 5; 2 6; 1 7;]+10

Z = zeros(20);
[m,n] = size(Y);
for i=1:m
    v = Y(i,:)
    Z(v(1), v(2)) = 1;
end

Y_2 = [1 1; 2 1; 3 1; 4 2; 4 3; 4 4; 4 5; 2 6; 1 7;]+10

center = repmat(mean(Y_2), length(Y_2), 1);
Y_2 = Y_2-center;
theta = 0.0;
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
Y_2 = R*Y_2'


Y_2 = Y_2'+center;

Y_2(:,1) = (Y_2(:,1));
Y_2(:,2) = Y_2(:,2);

Z_2 = zeros(20);
[m,n] = size(Y_2);
for i=1:m
    v = Y_2(i,:)
    Z_2(floor(v(1)), floor(v(2))) = 1;
end

X = [mean(Y); mean(Y_2);]

d1 = mahal(X, Y)

d2 = sum((X-repmat(mean(Y),2,1)).^2, 2)


c = corrcoef(Z, Z_2)

clf
figure(1)
plot(Y(:,1),Y(:,2))
hold on
plot(Y_2(:,1),Y_2(:,2))
hold on

scatter(Y(:,1),Y(:,2))
hold on

scatter(Y_2(:,1),Y_2(:,2))
hold on
scatter(X(:,1),X(:,2),100,d1,'*','LineWidth',2)
%scatter(X(:,1),X(:,2))
%hb = colorbar;
%ylabel(hb,'Mahalanobis Distance')
%legend('X','Y','Location','NW')