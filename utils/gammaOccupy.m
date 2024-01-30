function val = gammaOccupy(percentage, beta)
    if nargin == 1
        beta = 0.8; % desired percentage
    end

    if (percentage>= 0 && percentage <= beta)
        % val = 1/(1 + exp(-20 * (percentage - 0.25)));
        val = 1/(1 + exp(-20 * (percentage - 0.5)));
        % val = 0.3+0.7*(1/(1 + exp(-20 * (percentage - 0.5))));
        % val = 1/(1 + exp(-30 * (percentage - 0.6)));
        % val = 1/(1 + exp(-100 * (percentage - 0.7)));
    elseif (percentage <= 1)
        % val = -25 * percentage^2 + 40 * percentage -15;
        val = 1/(1 + exp(-30 * (-percentage + 1)));
    else
        warning("Percentage computed wrong in \gamma_occupy");
        keyboard; % this should never happen
    end
end
