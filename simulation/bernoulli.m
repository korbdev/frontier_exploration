%n = 50
%k = 50
%p_w = 0.9
%p_b = 0.1

%b = (p_w^k)*(1-p_w)^(n-k)

for i = 1:1000
    a = i;
    for j = 1:1000
        b = j;
        for k = 1:1000
            a = i;
        end
    end
end
