function dQ = quaternionDerivative(q, omega, k)
    dQ = 1/2 * [0       -omega'; ...
                omega   -skew(omega)] * q + k * (1 - q' * q) * q;
end
