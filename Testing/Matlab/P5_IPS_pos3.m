%% Recorded positions % DATA = IN % Transmitter 3 facing marks
m_0= [0 0; 0 1; -1 1];

m= [-459.07 -33.04; -457.13 -31.24; -456.01 -27.07];
m(:,:,2)= [-674.53 -39.96; -672.34 -38.73; -673.02 -38.72];
m(:,:,3)= [-883.99 -45.81; -881.79 -45.14; -884.15 -45.81];
m(:,:,4)= [-1086.32 -72.46; -1086.54 -73.14; -1086.14 -69.70];
m(:,:,5)= [-1296.11 -103.14; -1293.83 -114.05; -1295.50 -113.29];

gtruth= [-500 0; -700 0; -900 0; -1100 0; -1300 0];
%% Number crunching
for i=1:size(m,3) %Position index
    for j=1:size(m,1) %Measurement index
        accmat(i,j)= sqrt((m(j,1,i)-gtruth(i,1))^2 + (m(j,2,i)-gtruth(i,2))^2);
    end
end
accmat %(measurement, pos)

varvec= zeros([size(m,3) 1]);
varvec(:,:)=[];

for i=1:size(m,3) %Position index
     varvec(i)= var(accmat(i,:));
end
varvec %(trans, pos)

stdvvec= sqrt(varvec) %(trans, pos)

%Average standard deviation
avgStdv= sum(stdvvec)/size(stdvvec,2)

%Average accuracy
avgAcc= sum(sum(accmat))/(size(accmat,1)*size(accmat,3))

for i=1:size(accmat,1)
    avgaccvec(i)= sum(accmat(i,:))/size(accmat,2);
end
avgaccvec'