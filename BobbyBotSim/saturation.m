function sat = saturation(c)
    if (c < -1)
        sat = -1;
    elseif (c > 1)
        sat = 1;
    else
        sat = c;
    end
end
