file = load('data.txt');

desCOMx = file(:,19);
desCOMy = file(:,20);
lipmCOMx = file(:,17);
lipmCOMy = file(:,18);

ZMPx = file(:,1);
ZMPy = file(:,4);

RFx = file(:,8);
RFy = file(:,9);
RFz = file(:,10);
LFx = file(:,11);
LFy = file(:,12);
LFz = file(:,13);
% figure
% hold on;
% plot(ZMPy);
% plot(desCOMy);
% plot(lipmCOMy);
% plot(RFy);
% plot(LFy);
% legend('ZMP','desCOM','lipmCOM','RF','LF');

figure
hold on;
plot(ZMPx,ZMPy);
plot(desCOMx, desCOMy);
plot(lipmCOMx, lipmCOMy);
plot(RFx, RFy);
plot(LFx, LFy);
legend('ZMP','desCOM','lipmCOM','RF','LF');

figure
hold on;
plot(ZMPy);
% plot( desCOMy);
plot( lipmCOMy);

legend('ZMP','desCOM','lipmCOM','RF','LF');


figure
plot(RFy)
hold on
plot(LFy)
plot(ZMPy)

figure
plot(RFx)
hold on
plot(LFx)
plot(ZMPx)

figure
hold on;
plot(desCOMx);
plot(desCOMy);
legend('comx','comy');

figure
hold on;
plot(RFx);
plot(RFy);
plot(RFz);
legend('rfx','rfy','rfz');

figure
hold on;
plot(LFx);
plot(LFy);
plot(LFz);
legend('LLLfx','rfy','rfz');
figure
hold on;
plot(ZMPx);
% plot( desCOMy);
plot( lipmCOMx);

figure
plot(ZMPy);
hold on;

% plot( desCOMy);
plot( lipmCOMy);
plot(desCOMy);




