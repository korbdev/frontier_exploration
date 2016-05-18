function Y = gauss(X, mu, sigma)
    Y = zeros(size(X));
    kernel_size = 8;
    filter_kernel = normpdf([-kernel_size:kernel_size], mu, sigma);
    %filter_kernel = [1 2 4 2 1];
    top = X(1:kernel_size,:);
    bottom = X(end-kernel_size+1:end,:);

    extenden_vec = [bottom; X; top];

    for i=(kernel_size+1):size(X)+kernel_size
        v = extenden_vec(i-kernel_size:i+kernel_size);
        conv = filter_kernel*v;
        Y(i-kernel_size) = conv;
    end
end