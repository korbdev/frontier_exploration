function X_n = normalize(X)
X_min = min(X);
X_max = max(X);
X_n = (X- X_min)/(X_max - X_min);
end