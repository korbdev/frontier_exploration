x_bw = [10 20];
x_tw = [20 25];

%body frame
figure(1);
clf
hold on
grid on
xlim([-5 30]);
ylim([-5 30]);

plot(x_bw(1), x_bw(2), 'r.', 'MarkerSize', 20);
plot(x_tw(1), x_tw(2), 'r.', 'MarkerSize', 20);

plot([x_bw(1), x_bw(1)+5],[x_bw(2), x_bw(2)]);
plot([x_bw(1), x_bw(1)],[x_bw(2), x_bw(2)+5]);

%diff vec body origin - point
x_tbw = x_tw - x_bw
norm(x_tbw)

%Rotationswinkel rad
phi = 20/180*pi

%Rotationsmatrix
C_bw = [cos(phi), sin(phi); -sin(phi), cos(phi)]
%C_wb = C_bw'

%Einheitsvektor body frame
ex_b = [5 0]; ey_b = [0 5];

ex_bw = (C_wb*ex_b')';
ey_bw = (C_wb*ey_b')';

plot([0, ex_bw(1)],[0, ex_bw(2)],'g');
plot([0, ey_bw(1)],[0, ey_bw(2)],'g');

ex_bw = (C_wb*ex_b')'+x_bw;
ey_bw = (C_wb*ey_b')'+x_bw;

plot([x_bw(1), ex_bw(1)],[x_bw(2), ex_bw(2)],'g');
plot([x_bw(1), ey_bw(1)],[x_bw(2), ey_bw(2)],'g');

axis square

%point in rotated body frame
x_tb = (C_bw * (x_tw-x_bw)')'

%homogeneous vec point
xh_tw = [ x_tw 1]

tau = -C_bw*x_bw'

T_bw = [C_bw, tau; 0 0 1]

xh_tb = (T_bw*xh_tw')'