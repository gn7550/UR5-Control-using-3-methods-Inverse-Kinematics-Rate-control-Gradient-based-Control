function m = manipulability(J,measure)
% input: 6*6 Body Jacobian
% output: Body Jacobian Jbst
switch measure
    case 'sigmamin'
    m= svds(J,1,'smallest');
    case 'invcond'
    m= svds(J,1,'smallest')/svds(J,1);
    case 'detjac'
    m=det(J);
end
end