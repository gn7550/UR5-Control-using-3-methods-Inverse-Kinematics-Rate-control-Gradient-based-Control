function R = ROTZ(rad)

if ~isreal(rad)
        error('Input needs to be a real number');
end

R=[cos(rad) -sin(rad) 0; sin(rad) cos(rad) 0; 0 0 1];
