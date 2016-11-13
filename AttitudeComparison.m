if exist('atthist.mat', 'file') == 2
load('atthist.mat');
end
subplot(311)
plot(atthist(1,:), atthist(2,:),'r', atthist(1,:), atthist(5,:),'b');
xlim([atthist(1,1) atthist(1,end)])
legend('phic', 'phireal');
subplot(312)
plot(atthist(1,:), atthist(3,:),'r', atthist(1,:), atthist(6,:),'b');
xlim([atthist(1,1) atthist(1,end)])
legend('thetac', 'thetareal');
subplot(313)
plot(atthist(1,:), atthist(4,:),'r', atthist(1,:), atthist(7,:),'b');
xlim([atthist(1,1) atthist(1,end)])
legend('psic', 'psireal');
