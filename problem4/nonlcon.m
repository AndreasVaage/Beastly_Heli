function [c, c_eq] = nonlcon(z)
alpha = 0.2;
beta = 20;
lambda_t = 2 * pi / 3;

U = size(z, 1)  / 8;

lambda = z(1:6:6*U);
e = z(5:6:6*U);

c = zeros(U, 1);

for n = 1:U
    %c(n) = alpha * exp(-beta * (z((n - 1)*6 + 1) - lambda_t)^2) - z((n - 1)*6 + 5);
    c(n) = alpha * exp(-beta * (lambda(n) - lambda_t)^2) - e(n);
end

c_eq = [];

end

