function [X, P, E, Xc] = kalman_filter(Phi, X, P, Q, H, Z, R)

switch nargin
    case 4 %only time uopdate
        X = Phi*X;
        P = Phi*P*Phi' + Q;
    case 7 %time update and measure update
        X = Phi*X;
        P = Phi*P*Phi' + Q;
        K = P*H' / (H*P*H'+R);
        E = Z - H*X;
        Xc = K*E;
        X = X + Xc;
        In = eye(length(X));
        P = (In-K*H)*P*(In-K*H)' + K*R*K';
end

end