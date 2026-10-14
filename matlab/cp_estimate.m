function [s,z] = cp_estimate(truth,c,z)
% Encoder quantisation followed by the same two CV Kalman filters as Python.
res=[abs(c.hardware.cart_m_per_count);abs(c.hardware.pole_rad_per_count)];
y=[round(truth(1)/res(1))*res(1); ...
   round((truth(2)+pi)/res(2))*res(2)-pi];
r=res.^2/12; q=[2;0.3]; h=c.h; F=[1 h;0 1];
if isempty(z)
    z.x=[y';zeros(1,2)]; z.P=zeros(2,2,2);
    for j=1:2, z.P(:,:,j)=diag([r(j) 1]); end
end
for j=1:2
    xx=F*z.x(:,j); P=F*z.P(:,:,j)*F'+q(j)*[h^3/3 h^2/2;h^2/2 h];
    K=P(:,1)/(P(1,1)+r(j));
    z.x(:,j)=xx+K*(y(j)-xx(1)); z.P(:,:,j)=P-K*P(1,:);
end
s=[z.x(1,1);pi-mod(pi-z.x(1,2),2*pi);z.x(2,1);z.x(2,2)];
end
