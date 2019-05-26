function F = getFValue(f,x)

% R = @(t) 0;
% for k=1:f.p.pieces
%     D = f.p.coefs(k,1);
%     C = f.p.coefs(k,2);
%     B = f.p.coefs(k,3);
%     A = f.p.coefs(k,4);
% %     eval(['R',num2str(k),'= @(t) D*t^3 + C*t^2 + B*t + A;']);
%     piece = @(t) D*t^3 + C*t^2 + B*t + A;
%     R = @(t)(R(t)+piece(t));
% end

breaks = f.p.breaks;
if breaks(1)<x && x<breaks(2)
    fprintf('in first section');
    D = f.p.coefs(1,1);
    C = f.p.coefs(1,2);
    B = f.p.coefs(1,3);
    A = f.p.coefs(1,4);
    F = D*x^3 + C*x^2 + B*x + A;
elseif breaks(2)<x && x<breaks(3)
    fprintf('in second section');
    D = f.p.coefs(2,1);
    C = f.p.coefs(2,2);
    B = f.p.coefs(2,3);
    A = f.p.coefs(2,4);
    F = D*x^3 + C*x^2 + B*x + A;
elseif breaks(3)<x && x<breaks(4)
    fprintf('in third section');
    D = f.p.coefs(3,1);
    C = f.p.coefs(3,2);
    B = f.p.coefs(3,3);
    A = f.p.coefs(3,4);
    F = D*x^3 + C*x^2 + B*x + A;
elseif breaks(4)<x && x<breaks(5)
    fprintf('in fourth section');
    D = f.p.coefs(4,1);
    C = f.p.coefs(4,2);
    B = f.p.coefs(4,3);
    A = f.p.coefs(4,4);
    F = D*x^3 + C*x^2 + B*x + A;
else
    error('given point is not within spline range');
end