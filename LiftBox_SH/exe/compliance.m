file = load('WBWALK.txt');

rfz = file(:,235);
lpf_rfz = file(:,236);
posx = file(:,237);
measuredz = file(:,238);

rsp = file(:,241);
rhx = file(:,185);
rhy = file(:,186);
rhz = file(:,187);

figure
hold on;
plot(rfz);
plot(lpf_rfz);
plot(posx);
plot(measuredz);

legend('rfz','lpf','posx','measuredz');

figure
hold on;
plot(rsp);
plot(rhx);
plot(rhy);
plot(rhz);
legend('rsp','rhx','y','z');