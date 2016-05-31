%d = [80 40 10]';
%c = [100 40 10]';

%d_pmt = perms(d)
%c_pmt = perms(c)

%omega = 0.5;
%theta = 0.5;

%d_norm = d_pmt./max(d_pmt(:))
%c_norm = c_pmt./max(c_pmt(:))

%s = omega*(1./d_norm)+theta*c_norm

%s_final = [s d_pmt c_pmt]

d_c = [80 100; 40 40; 10 25];

omega = 0.1;
theta = 0.1;

d_c_perm = perms(d_c);

[m, n] = size(d_c_perm)

d = d_c_perm(:,1:n/2);
c = d_c_perm(:,(n/2)+1:n);

for i=1:m
    d_norm(i,:) = d(i,:)./max(d(i,:));
    c_norm(i,:) = c(i,:)./max(c(i,:));
end

s = omega*(1./d_norm)+theta*c_norm;

s_final = [s d_norm c_norm]

x_2 = 0.125;
y_2 = 0.1;
str2 = sprintf('x=%f, y=%f', x_2, y_2);

x_3 = 0.3;
y_3 = 1;
str3 = sprintf('x=%f, y=%f', x_3, y_3);

figure(1)
subplot(2, 3, 1)
[X,Y] = meshgrid(0:0.07:1, 0:0.1:1);
%Z = X .* exp(-X.^2 - Y.^2);
%Z = omega*(1./X)+theta*Y;
%Z = (1-omega*X)+theta*Y;
Z = exp(1./X).*exp(Y);
surf(X,Y,Z)

title('f(d,c)');
xlabel('distance')
ylabel('cardinality')

x = 0.9;
y = 1;
str = sprintf('x=%f, y=%f', x_2, y_2);

subplot(2,3, 2)
[O,T] = meshgrid(0:1, 0:1);
%P = O*(1/0.9)+T*1;
contour(O, T, O*(1/x_2)+T*y_2)
title(str);
xlabel('omega')
ylabel('theta')
colorbar
%surf(O,T,P)



x = 0.125;
y = 1;
str = sprintf('x=%f, y=%f', x_3, y_3);
subplot(2,3, 3)
[O,T] = meshgrid(0:1, 0:1);
%P = O*(1/0.9)+T*1;
contour(O, T, O*(1/x_3)+T*y_3)
title(str);
axis('tight')
xlabel('omega')
ylabel('theta')
colorbar


%x_1 = 1;
%y_1 = 1;
%str1 = sprintf('x=%f, y=%f', x_1, y_1);

%x_4 = 0.125;
%y_4 = 0.1;
%str4 = sprintf('x=%f, y=%f', x_4, y_4);

subplot(2,3, 4)
%figure(2)
[O,T] = meshgrid(0:1, 0:1);
%P = O*(1/x_1)+T*y_1
%s1 = surf(O,T,P, 'FaceColor', [255,0,0]/255)
%alpha(s1, 0.3)

hold on

Q = O*(1/x_2)+T*y_2
s2 = surf(O,T,Q,'FaceColor', [255,255,0]/255)
alpha(s2, 0.3)

hold on

Q = O*(1/x_3)+T*y_3
s3 = surf(O,T,Q,'FaceColor', [0,0,255]/255)
alpha(s3, 0.3)

%hold on

%Q = O*(1/x_4)+T*y_4
%s4 = surf(O,T,Q,'FaceColor', [0,255,255]/255)
%alpha(s4, 0.3)

title(str);
xlabel('omega')
ylabel('theta')

legend([s2, s3], {str2, str3});
%title(str);
%xlabel('omega')
%ylabel('theta')
%zlim(0:20)
