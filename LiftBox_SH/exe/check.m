file = load('data.txt');
close all
startT = 1;
stopT = 15000;
Time = [startT:1:stopT];
win0zmpx = file(Time,1);
win0zmpy = file(Time,4);
win299zmpx = file(Time,3);
win299zmpy = file(Time,6);

win0rfx = file(Time,8);
win0rfy = file(Time,9);
win0lfx = file(Time,11);
win0lfy = file(Time,12);

lipmx = file(Time,17);
lipmy = file(Time,18);
descomx = file(Time,19);
descomy = file(Time,20);

target0rfx = file(Time,92);
target0lfx = file(Time,89);
target0rfy = file(Time,93);
target0lfy = file(Time,90);

target1rfx = file(Time,209);
target1lfx = file(Time,211);
target1rfy = file(Time,210);
target1lfy = file(Time,212);

target2rfx = file(Time,213);
target2lfx = file(Time,215);
target2rfy = file(Time,214);
target2lfy = file(Time,216);
time = file(Time,207);

sh0r = file(Time,217);
sh0l = file(Time,218);
sh1r = file(Time,219);
sh1l = file(Time,220);
sh2r = file(Time,221);
sh2l = file(Time,222);
sh3r = file(Time,223);
sh3l = file(Time,224);

tar0mov = file(Time,225);
tar1mov = file(Time,226);
tar2mov = file(Time,227);


% figure
% hold on;
% plot(win0zmpx);
% plot(target0rfx);
% plot(target0lfx);
% legend('zmp','rf','lf');

figure
hold on;
plot(win0zmpx);
plot(win299zmpx);
plot(lipmx);
legend('win0zmpx','win299zmpx','lipmx');

figure
hold on;
plot(win0zmpx, win0zmpy);
% plot(win299zmpx, win299zmpy);
plot(lipmx, lipmy);
plot(descomx, descomy);
legend('zmp0','lipm','des');

figure
hold on;
plot(win0zmpy);
plot(win299zmpy);
plot(lipmy);
legend('win0zmpy','win299zmpy','lipmy');

% figure
% hold on;
% plot(target0rfx);
% plot(target1rfx);
% plot(target2rfx);
% plot(win0zmpx);
% legend('tar0rfx','tar1rfx','tar2rfx','t','zmp');
% 
% figure
% subplot(2,1,1);
% hold on;
% plot(target0rfx);
% plot(target1rfx);
% plot(target2rfx);
% legend('RR0x','1x','2x');
% subplot(2,1,2);
% hold on;
% plot(target0lfx);
% plot(target1lfx);
% plot(target2lfx);
% legend('LL0x','1x','2x')
% 
% figure
% subplot(2,1,1);
% hold on;
% plot(sh0r);
% plot(sh1r);
% plot(sh2r);
% plot(sh3r);
% legend('0','1','2','3');
% subplot(2,1,2);
% hold on;
% plot(sh0l);
% plot(sh1l);
% plot(sh2l);
% plot(sh3l);
% legend('0','1','2','3');
% 
% figure
% hold on;
% plot(tar0mov);
% plot(tar1mov);
% plot(tar2mov);
% legend('mov0','mov1','mov2');
% 
% 
% figure
% plot(file(Time,7))