figure(1)
%subplot(2, 3, 1)
x = linspace(0,80);
a = exp(-20*(1/10));
b = cos(x);
plot(x,a)

title('f(d,c)');
xlabel('distance')
ylabel('cardinality')