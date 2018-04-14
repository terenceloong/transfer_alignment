%generate drift data
function data = gene_drift(n, m, dt, tr, fw, gain)
%n-row, m-column, dt-sample time, tr-correlation time, fw-filter parameter [fw1,fw2]

%--generate random sequence which correlation time is tr--%
n = n + 100/dt; %100s is used to make filter stable
data = zeros(n,m);
data(1,:) = randn(1,m);
for k=2:n
    data(k,:) = data(k-1,:);
    if mod(k-1,tr/dt)==0
        data(k,:) = randn(1,m);
    end
end

%--two levels filter--%
for p=1:2
    fc = 1/tr/fw(p);
    tao = 1/(2*pi*fc);
    x0 = data(1,:);
    data(1,:) = zeros(1,m);
    for k=2:n
        x = data(k,:);
        y0 = data(k-1,:);
        y = (dt*(x+x0)-(dt-2*tao)*y0)/(dt+2*tao); %one order filter
        x0 = x;
        data(k,:) = y;
    end
end

data = data(100/dt+1:end,:)*gain;

end