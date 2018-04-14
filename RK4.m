function x1 = RK4(fun, x0, dt, varargin)

switch nargin
    case 4
        t = varargin{1};
        K1 = fun(x0, t);
        K2 = fun(x0+K1*dt/2, t+dt/2);
        K3 = fun(x0+K2*dt/2, t+dt/2);
        K4 = fun(x0+K3*dt, t+dt);
        x1 = x0 + (K1+2*K2+2*K3+K4)*dt/6;
    case 6
        u0 = varargin{1};
        u1 = varargin{2};
        u2 = varargin{3};
        K1 = fun(x0, u0);
        K2 = fun(x0+K1*dt/2, u1);
        K3 = fun(x0+K2*dt/2, u1);
        K4 = fun(x0+K3*dt, u2);
        x1 = x0 + (K1+2*K2+2*K3+K4)*dt/6;
    case 7
        t  = varargin{1};
        u0 = varargin{2};
        u1 = varargin{3};
        u2 = varargin{4};
        K1 = fun(x0, t, u0);
        K2 = fun(x0+K1*dt/2, t+dt/2, u1);
        K3 = fun(x0+K2*dt/2, t+dt/2, u1);
        K4 = fun(x0+K3*dt, t+dt, u2);
        x1 = x0 + (K1+2*K2+2*K3+K4)*dt/6;
end

end