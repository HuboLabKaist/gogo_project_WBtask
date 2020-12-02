data = load('data.txt');

Px = data(:,11);
Py = data(:,12);
Pth = data(:,13);

Vx = data(:,17);
Vy = data(:,18);
Vth = data(:,19);

Tcur = data(:,21);
Tgoal = data(:,22);
Tsat = data(:,23);

figure
hold on;
plot(Vx);
%plot(Vy);
%plot(Vth);
plot(Px);
%plot(Py);
%plot(Pth);
legend('Vx','Vy','Vth')

