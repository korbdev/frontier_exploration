N = [20,20];
p = 0.33;
alpha = 0.9;
region = ones(N);
region(10,1:9) = 0;

pi = zeros(N);
pi(3,3) = 1; region(3,3) = 0;
pi(3,4) = 1; region(3,4) = 0;


figure(1)
clf
for k=1:100
    imagesc(pi)
    colorbar
    drawnow
    pi=iterate(pi,region,p,alpha)
end
