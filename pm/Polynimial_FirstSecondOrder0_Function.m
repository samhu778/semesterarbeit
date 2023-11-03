function [f, df, ddf] = Polynimial_FirstSecondOrder0_Function(t, t0, te, x0, xe)
    
    n = length(x0);
    f = zeros(n, 1);
    df = zeros(n, 1);
    ddf = zeros(n, 1);
    
    for i = 1:n
        a0 = (xe(i) * t0^5 - 5 * xe(i) * t0^4 * te + 10 * xe(i) * t0^3 * te^2 ...
            - 10 * x0(i) * t0^2 * te^3 + 5 * x0(i) * t0 * te^4 - x0(i) * te^5) / (t0 - te)^5;
        a1 = (30 * t0^2 * te^2 * (x0(i) - xe(i))) / (t0 - te)^5;
        a2 = -(30 * t0 * te * (t0 + te) * (x0(i) - xe(i))) / (t0 - te)^5;
        a3 = (10 * (x0(i) - xe(i)) * (t0^2 + 4 * t0 * te + te^2)) / (t0 - te)^5;
        a4 = -(15 * (t0 + te) * (x0(i) - xe(i))) / (t0 - te)^5;
        a5 = (6 * (x0(i) - xe(i))) / (t0 - te)^5;
        p = [a5, a4, a3, a2, a1, a0];
        dp = [5 * a5, 4 * a4, 3 * a3, 2 * a2, a1];
        ddp = [20 * a5, 12 * a4, 6 * a3, 2 * a2];

        f(i) = p(1) * t^5 + p(2) * t^4 + p(3) * t^3 + p(4) * t^2 + p(5) * t + p(6);
        df(i) = dp(1) * t^4 + dp(2) * t^3 + dp(3) * t^2 + dp(4) * t + dp(5);
        ddf(i) = ddp(1) * t^3 + ddp(2) * t^2 + ddp(3) * t + ddp(4);
    end
end