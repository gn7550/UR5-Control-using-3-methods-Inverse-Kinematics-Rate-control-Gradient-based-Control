function R = ROTX(rad)

if ~isreal(rad)
        error('Input needs to be a real number');
end

R=[1 0 0; 0 cos(rad) -sin(rad); 0 sin(rad) cos(rad)];
