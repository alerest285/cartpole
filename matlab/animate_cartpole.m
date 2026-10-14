function animate_cartpole(result)
d=result.data; len=2*result.config.plant.l;
figure('Name',['Cart-pole: ' result.mode],'Color','w');
subplot(2,1,1); hold on; axis equal; axis([-0.45 0.45 -0.4 0.4]); grid on;
plot([-0.2 0.2],[0 0],'k-');
cart=plot(0,0,'s','MarkerSize',18,'MarkerFaceColor',[0.1 0.4 0.7]);
pole=plot([0 0],[0 len],'-o','LineWidth',3,'Color',[0.9 0.4 0.1]);
label=title(''); xlabel('Position [m]'); ylabel('Height [m]');
subplot(2,1,2); plot(d(:,1),d(:,[2 11])); grid on;
legend('Cart [m]','Voltage [V]'); xlabel('Time [s]');
started=tic;
for k=1:10:size(d,1)
    if ~isgraphics(cart), break; end
    x=d(k,2); a=d(k,3);
    set(cart,'XData',x); set(pole,'XData',[x x-len*sin(a)],'YData',[0 len*cos(a)]);
    set(label,'String',sprintf('%s | %.2f s | %.2f V',result.mode,d(k,1),d(k,11)));
    drawnow; pause(max(0,d(k,1)-toc(started)));
end
end
