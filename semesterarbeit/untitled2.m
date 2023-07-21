q1_t1 = input
q2_t2 = input

function[ h1,h2 ] = calculate(q1_t, q2_t, q1_t1, q2_t2, delta_t)
    diff = (q1_t - q1_t1)/delta_t
    q1_t1 = q1_t
    q2_t2 = q2_t
end