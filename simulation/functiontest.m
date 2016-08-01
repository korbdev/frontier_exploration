% cbm, 2003/12/03
% public domain, no rights reserved, share with anyone

th = 0:(2*pi/100):2*pi;
r = 0:.2:1;

% create a polar grid
[thg, rg] = meshgrid(th,r);

% you MUST use ".^", "./" and ".*" here to "vectorize" the code:
%f = inline('cos(5*th) + 6*exp(-r.^2)', 'th', 'r');
f = inline('cos(2*th)', 'th', 'r');
u = feval(f,thg,rg);

% here's the magic part...
[x,y,z] = pol2cart(thg,rg,u);

% or try pcolor or contour
figure(2); clf;
surf(x,y,z);
%pcolor(x,y,z);
%contour(x,y,z);
xlabel('x');
ylabel('y');
zlabel('z');


figure(3);clf;
fplot(@(x) -cos(2*x), [0 2*pi])
% this turns off the grid lines (works in matlab only)
%shading interp