%% Recorded positions % DATA = MISSING
m_0= [0 0; 0 1; -1 1];

m= [-445.73 -43.65; -446.56 -44.86; -447.21 -43.05];
m(:,:,2)= [-653.15 -15.80; -653.27 4.44; -651.19 -7.59];
m(:,:,3)= [-865.56 -42.23; -866.46 -43.54; -866.30 -42.88];
m(:,:,4)= [-1063.19 -72.22; -1062.41 -73.63; -1063.37 -74.31];
m(:,:,5)= [-1294.49 -99.22; -1283.40 -105.89; -1293.24 -97.00];
%gtruth= [500 0; 700 0; 900 0];
gtruth= [-500 0; -700 0; -900 0; -1100 0; -1300 0]; % For 3rd transmitter

%% Number crunching
avg= sum(m,2)/size(m,2); %sum all measurements of a transmitter, divide by number of measurements
accmat= zeros([size(m,1),size(m,3)]);
accmat(:,:)=[];

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