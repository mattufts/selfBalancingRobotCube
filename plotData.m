data = readtable("data.csv");

figure;
hold on;
plot(data.Var1,"DisplayName","IMU angle");
plot(data.Var2,"DisplayName","Lidar angle");
hold off;
xlabel("time step (n)")
ylabel("sensor reading (degrees)")
