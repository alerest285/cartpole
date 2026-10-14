function [v,state,phase] = cp_lab_controller(y,manual,arm,xref,cap,mode,physical,gain,settings)
%#codegen
% Fixed-size 500 Hz controller for copied course QUARC models.
% Modes: 1 LQR, 2 poles, 3 PID, 4 mouse, 5 fixed sine, 6 energy,
% 7 identification, 8 phase-locked sine. Original I/O/watchdog remain outside.
% settings: catch/drop deg, ke, amax, kx, kv, eps, kick, Eover, adapt,
%           amplitude V, frequency Hz, phase lead deg, Ki angle, Ki position.
persistent hat covariance previous_angle angle_u active integral tick bias previous_ad fault
h=0.002; r=[(2.2749e-5)^2;(2*pi/4096)^2]/12;
if isempty(hat)
    hat=zeros(2,2); hat(1,:)=y(:)'; covariance=zeros(2,2,2);
    for j=1:2, covariance(:,:,j)=diag([r(j) 1]); end
    previous_angle=y(2); angle_u=y(2); active=false; integral=zeros(2,1);
    tick=0; bias=0; previous_ad=0; fault=false;
end
v=0; phase=0; state=[y(1);y(2);0;0];
if any(~isfinite(y))||~isfinite(cap)||~isfinite(manual)||~isfinite(xref)
    fault=true; phase=-1; return;
end
angle_u=angle_u+atan2(sin(y(2)-previous_angle),cos(y(2)-previous_angle)); previous_angle=y(2);
measurement=[y(1);angle_u]; F=[1 h;0 1]; q=[2;0.3];
for j=1:2
    pred=F*hat(:,j); P=F*covariance(:,:,j)*F'+q(j)*[h^3/3 h^2/2;h^2/2 h];
    L=P(:,1)/(P(1,1)+r(j)); hat(:,j)=pred+L*(measurement(j)-pred(1));
    covariance(:,:,j)=P-L*P(1,:);
end
s=[hat(1,1);pi-mod(pi-hat(1,2),2*pi);hat(2,1);hat(2,2)];
state=[y(1);y(2);s(3);s(4)]; % raw positions for offline system identification
if arm<0.5
    active=false; integral(:)=0; tick=0; bias=0; previous_ad=0; fault=false; return;
end
if abs(y(1))>0.19||abs(s(1))>0.19||abs(s(4))>60||any(~isfinite(s)), fault=true; end
if fault, phase=-1; return; end
cap=max(0,min(10,cap)); x=s(1)-xref; a=s(2); xd=s(3); ad=s(4);
old_integral=integral; wanted=0;
if mode<=3
    threshold=5*pi/180; if active, threshold=10*pi/180; end
    active=abs(a)<threshold && abs(y(2))<10*pi/180; % preserve the course angle gate
    if active
        phase=3; integral=integral+h*[-a;-x];
        wanted=-gain*[x;a;xd;ad];
        if mode==3, wanted=wanted+settings(14)*integral(1)+settings(15)*integral(2); end
    else
        phase=1; integral(:)=0;
    end
elseif mode==7
    phase=6; tau=mod(tick*h,20);
    wanted=settings(11)*sin(2*pi*(0.3*tau+0.5*(3-0.3)/20*tau^2))-25*x-2*xd;
else
    if active && abs(a)>settings(2)*pi/180, active=false;
    elseif ~active && abs(a)<settings(1)*pi/180, active=true; end
    if active
        phase=3; wanted=-gain*[x;a;xd;ad];
    else
        phase=2;
        if mode==4
            wanted=manual;
        elseif mode==5
            wanted=settings(11)*sin(2*pi*settings(12)*tick*h)-5*x-xd;
        elseif mode==8
            energy=0.5*physical(2)*ad^2/(physical(3)*physical(4))+1+cos(a);
            th=acos(max(-1,min(1,1-energy))); ag=1; bg=sqrt(max(1e-12,1-sin(th/2)^2));
            for j=1:30, old=ag; ag=(ag+bg)/2; bg=sqrt(old*bg); end
            w=2*pi*settings(12)*ag; phi=pi-mod(pi-(a-pi),2*pi);
            psi=atan2(-ad/w,phi); if th<2*pi/180, psi=2*pi*settings(12)*tick*h; end
            wanted=settings(11)*sin(psi+settings(13)*pi/180)-5*x-xd;
        else
            E=0.5*physical(2)*ad^2/(physical(3)*physical(4))+cos(a)-1;
            if previous_ad*ad<0 && abs(a)<pi/2 && abs(a)>settings(1)*pi/180
                bias=min(0.5,bias+settings(10)*(1-cos(a))*0.5);
            end
            previous_ad=ad; w=ad*cos(a);
            if abs(ad)<0.3 && abs(a)>170*pi/180 && E< -1.9, pump=settings(8);
            else, pump=settings(3)*(settings(9)+bias-E)*w; end
            desired=max(-settings(4),min(settings(4),pump))+ ...
                (-settings(5)*x-settings(6)*xd)*settings(7)/(w*w+settings(7));
            a0=cart_accel(s,0,physical); a1=cart_accel(s,1,physical);
            wanted=(desired-a0)/(a1-a0);
        end
    end
end
tick=tick+1;
a0=cart_accel(s,0,physical); beta=cart_accel(s,1,physical)-a0;
lo=max(-cap,(-144*(0.17+s(1))-24*xd-a0)/beta);
hi=min(cap,(144*(0.17-s(1))-24*xd-a0)/beta);
if lo>hi, lo=max(-cap,min(cap,(lo+hi)/2)); hi=lo; end
v=max(lo,min(hi,wanted));
if mode==3 && active
    increment=settings(14)*(integral(1)-old_integral(1))+settings(15)*(integral(2)-old_integral(2));
    if (wanted-v)*increment>0, integral=old_integral; end
end
if ~isfinite(v), v=0; fault=true; phase=-1; end
end

function xdd=cart_accel(s,v,p)
% physical = [M J ml g a b Bf Bp Fcoul kamp]
cs=cos(s(2)); sn=sin(s(2));
force=p(10)*p(5)*v-(p(6)+p(7))*s(3)-p(9)*tanh(s(3)/0.01);
r1=force-p(3)*sn*s(4)^2; r2=p(3)*p(4)*sn-p(8)*s(4);
xdd=(p(2)*r1+p(3)*cs*r2)/(p(1)*p(2)-(p(3)*cs)^2);
end
