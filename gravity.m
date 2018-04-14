%calculate gravity, lat:rad

function g = gravity(lat, h)

global a f w GM re rp
b = (1-f)*a;
k = b*rp/(a*re)-1;
m = w*w*a*a*b/GM;
e2 = f*(2-f);
r = re * (1+k*sin(lat)^2) / (1-e2*sin(lat)^2)^0.5;
g = r * (1 - 2/a*(1+f+m-2*f*sin(lat)^2)*h + 3/a^2*h^2);
    
end