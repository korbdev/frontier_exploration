

p = [1/3 2/3; 1 0; 0 1;]

e = -sum(p.*log2inf(p))

H = sum(e)
