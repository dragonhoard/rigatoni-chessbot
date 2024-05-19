function value = computeNeglect(value, tiny)
% checks if value is less than neglectable value
% will set to 0
% intended to get rid of tiny computation values

if abs(imag(value)) < tiny
    value = real(value);
end
if abs(value) < tiny
    value = 0;
end
end