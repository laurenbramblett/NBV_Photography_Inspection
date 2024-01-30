function val = gammaPerspective(x)
    if (x >= 0 && x <= 1)
        % val = 0.3 + 0.7/(1+exp(10*(x-0.65))); % <====== using this for sim now
        val = 0.3 + 0.7/(1+exp(20*(x-0.75)));
        % val = 1/(1+exp(20*(x-0.75)));
        % val = 1/(1+exp(10*(x-0.65)));
        % val = 1/(1+exp(20*(x-0.5)));
        % val = 1/(1+exp(20*(x-0.3)));
        % val = 1/(1+exp(20*(x-0.25)));
    else
        warning("wrong \gamma_{perspective} computed!");
        keyboard; % This should never happen
    end
end
