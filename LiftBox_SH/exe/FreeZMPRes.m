clc
close all
clear 
data = load('data.txt');

figure
plot(data(:,63))

figure
plot(data(:,4))
hold on
plot(data(:,18))
plot(data(:,20))
plot(data(:,64)/1000)
plot(data(:,24)*0.1)
plot(data(:,97)*0.1)






figure
plot(data(:,62))
hold on
plot(data(:,64)/1000)

figure
plot(data(:,185))
hold on
plot(data(:,187))
plot(data(:,167))
plot(data(:,97))
plot(data(:,192))
plot(data(:,193))

legend('ratio','des right roll torque','cpy','IMU roll','LPF_roll','LPF_roll_vel')


figure
plot(data(:,97))
hold on
plot(data(:,98))
legend('roll','vel')

figure
plot(data(:,119))
hold on
plot(data(:,179))
plot(data(:,187))
plot(data(:,198))
legend('RD roll','Mx','des Mx','LPF Mx')

figure
plot(data(:,119))
hold on
plot(data(:,179))
plot(data(:,187))
plot(data(:,198))
legend('LD roll','Mx','des Mx','LPF Mx')


figure
plot(data(:,103))
hold on
plot(data(:,180))
plot(data(:,189))
legend('RD pitch','My','des My')

a = 0.15;
frq = a/((2*3.141592*(1-a))*0.005)

figure
hold on
plot(data(:,167))
plot(data(:,97))
plot(data(:,192))
plot(data(:,193))
legend('CPy','Roll','LPF_Roll','LPF_vel')

figure
hold on
plot(data(:,194))
plot(data(:,73)*3.141592/180)
plot(data(:,190))
plot(data(:,191))
plot(data(:,4))
legend('CPx','PitchComp','LPF_Pitch','LPF_vel')


datain = data(1:2500,74);
fc = [2.5 3.5];
fs = 200;
[b,a] = butter(1,fc/(fs/2),'stop');
figure
freqz(b,a)

dataout = filter(b,a,datain);
figure
plot(dataout)

[A,B,C,D] = butter(1,fc/(fs/2),'stop')

figure
plot(data(:,246))


figure
plot(data(:,63))


figure
plot(data(:,194))
hold on
plot(data(:,228))
plot(data(:,7))
plot(data(:,230),'-.')
plot(data(:,231),'--')
legend('cpx','cpx dot','state','hiptorque','hip ref')


figure
plot(data(:,166))
hold on
plot(data(:,228))
plot(data(:,166)*100 + data(:,228)*1.0)
legend('cpx','cpx dot','hip torque')

figure
plot(data(:,231))


figure
plot(data(:,95))
legend('pitch')


figure
plot(data(:,96))
legend('pitch vel')
%    -0.5450 --y





