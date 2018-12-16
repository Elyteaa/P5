%%Recorded data and grount truth % DATA = IN
%Moving, then reading % 90, 180, 270, 360 degrees
m= [-1 4 0;      %90
        -40 -34 -33; %180
        -30 -36 -37; %270
        1 5 0];      %360

%Moving according to IMU % 90, 180, 270, 360
%To see data for this, name it 'm'
m_2= [5 -3 -8;  %90
      1 0 3;    %180
      10 4 3;   %270
      4 6 4];   %360
%% Number scronching
for i=1:size(m,1)
    accVec(i)= abs(sum(m(i,:))/size(m,2));
end
accVec

for i=1:size(m,1)
    varVec(i)= var(m(i,:));
end
varVec

stdvVec= sqrt(varVec);
stdvVec