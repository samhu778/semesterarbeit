function R = get_R(phi)
   

    skew_phi = skew(phi);
    theta = norm(phi);

    if theta ~= 0
        R = eye(3) + sin(theta) / theta * skew_phi + ...
            (1 - cos(theta)) / (theta^2) * (skew_phi * skew_phi);
    else
        R = eye(3);
    end

    % Note that Rodrigues' formula has an undefined but continuous point at phi = 0
    % Additional definition: R(phi = 0) = I, to make it globally continuous
end

function S = skew(v)
    
    S = [0, -v(3), v(2);
         v(3), 0, -v(1);
         -v(2), v(1), 0];
end