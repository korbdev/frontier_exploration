function z = log2inf(m)
    z = log2(m);
    i = find(z==-Inf)
    z(i) = 0;
    i = find(z==Inf)
    z(i) = 0;
end