function R = ROTY(rad)

if ~isreal(rad)
        error('Input needs to be a real number');
end

R=[cos(rad) 0 sin(rad); 0 1 0; -sin(rad) 0 cos(rad)];
