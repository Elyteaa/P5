accavg= [25.1 23.1 28.3 30.3 384.1;
            43.5 55.5 78.0 96.1 120.6;
            52.4 47.4 48.6 74.1 110.3;
            44.6 61.4 13.2 13.8 28.0];
x= [0 20 40 60 80];

stdvavg= [1.01 0.798 1.84 2.42 343.57;
          0.39 1.16 9.10 2.21 2.17;
          0.712 0.200 0.103 1.75 6.13;
          1.65 4.58 0.528 0.391 5.19];

%plot(x, accavg(1,:), x, accavg(2,:), x, accavg(3,:), x, accavg(4,:))

plot(x, stdvavg(1,:), x, stdvavg(2,:), x, stdvavg(3,:), x, stdvavg(4,:))
axis([0 80 0 10])