function pinew = iterate(pi,region,p,alpha)
pinew = zeros(size(pi));
for x=2:size(pi,1)-1
    for y=2:size(pi,2)-1
        if region(x,y)==1
            a = pi(x-1,y); b = pi(x,y+1); c = pi(x+1,y); d = pi(x,y-1);
            preach = p*a + (1-p*a)*p*b + (1-p*a)*(1-p*b)*p*c + (1-p*a)*(1-p*b)*(1-p*c)*p*d;
            pinew(x,y) = (1-alpha)*pi(x,y) + alpha*preach;
        else
            pinew(x,y) = pi(x,y);
        end
    end
end
end

