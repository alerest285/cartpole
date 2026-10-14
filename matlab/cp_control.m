function [v,z] = cp_control(s,t,mode,c,z)
% Controller core. z is explicit state: no persistent state between runs.
if isempty(z)
    z=struct('active',false,'phase',0,'last_t',t,'k',0,'u',0);
end
s=s(:); x=s(1)-c.x_ref; a=s(2); xd=s(3); ad=s(4); p=c.plant;
switch char(mode)
    case 'sine_test'
        v=5*sin(2*pi*10*t);
    case {'lqr','place','pid'}
        limit=5*pi/180;
        if z.active
            limit=30*pi/180;
            if strcmp(mode,'pid'), limit=25*pi/180; end
        end
        z.active=abs(a)<limit;
        v=0;
        if z.active
            if strcmp(mode,'pid'), K=[-10 42 -16 5.5];
            else, K=c.gains.(char(mode)); end
            v=-K*[x;a;xd;ad];
        end
    case {'energy','sine'}
        if z.active && abs(a)>45*pi/180, z.active=false;
        elseif ~z.active && abs(a)<35*pi/180, z.active=true; end
        if strcmp(mode,'sine')
            e=0.5*p.J*ad^2/(p.ml*p.g)+1+cos(a);
            th=acos(max(-1,min(1,1-e)));
            ag=1; bg=sqrt(max(1e-12,1-sin(th/2)^2));
            for j=1:30, old=ag; ag=(ag+bg)/2; bg=sqrt(old*bg); end
            w=2*pi*1.05*ag;
            phi=pi-mod(pi-(a-pi),2*pi);
            psi=atan2(-ad/w,phi);
            if th<2*pi/180, psi=z.phase; end
            z.phase=z.phase+w*(t-z.last_t); z.last_t=t;
            v=8*sin(psi-40*pi/180)-5*x-xd;
        else
            E=0.5*p.J*ad^2/(p.ml*p.g)+cos(a)-1;
            w=ad*cos(a);
            if abs(ad)<0.3 && abs(a)>170*pi/180 && E< -1.9
                pump=2;
            else
                pump=5*(-E)*w;
            end
            desired=max(-5,min(5,pump))+(-2*x-2*xd)*3/(w*w+3);
            a0=cp_deriv(s,0,p); a1=cp_deriv(s,1,p);
            v=(desired-a0(3))/(a1(3)-a0(3));
        end
        if z.active, v=-c.gains.(char(mode))*[x;a;xd;ad]; end
    case 'ppo'
        if mod(z.k,10)==0
            o=[x/0.2 sin(a) cos(a) xd ad/10 z.u];
            q=c.policy;
            h1=tanh(o*q.W1+q.b1(:)');
            h2=tanh(h1*q.W2+q.b2(:)');
            z.u=max(-1,min(1,h2*q.W3+q.b3));
        end
        z.k=z.k+1; v=z.u*c.policy.v_max;
        z.active=abs(a)<12*pi/180;
    otherwise
        error('cartpole:mode','Unknown controller %s',mode);
end
end
