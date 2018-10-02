function line = demo(dt)
%dt=0.002;
coe=14.6884 *dt;
K=0.008;
raw=load('imu.txt');
line=[raw,zeros(length(raw),2)];
%line=k;
line(:,2)=line(:,2)/coe;
line(1,3)=line(1,1);
line(1,4)=line(1,1);
for i=2:length(line)
    line(i,3)=line(i-1,3)-dt*line(i,2);
    line(i,4)=(line(i-1,4)-dt*line(i,2))*(1-K)+line(i,1)*K;
end
plot(line);
end