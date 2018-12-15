%% Recorded positions % DATA = IN % Transmitter 2 facing marks
m_0= [0 0; 0 1; -1 1];

m= [43.70 -501.72; 43.69 -502.79; 43.03 -502.09];
m(:,:,2)= [55.69 -707.68; 55.06 -710.66; 53.77 -706.28];
m(:,:,3)= [65.72 -915.76; 66.40 -947.95; 67.76 -950.45];
m(:,:,4)= [95.95 -1121.91; 91.72 -1120.70; 93.11 -1122.89];
m(:,:,5)= [111.30 -1351.41; 109.02 -1352.25; 107.53 -1349.31];

gtruth= [0 -500; 0 -700; 0 -900; 0 -1100; 0 -1300]; % For 2nd transmitter
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