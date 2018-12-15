%% Recorded positions % DATA = IN % Transmitter 1 facing marks
m_0= [0 0; 0 1; -1 1];

m= [475.12 -8.53; 476.83 -8.53; 477.11 -8.54];
m(:,:,2)= [717.37 -15.80; 716.81 -14.54; 718.21 -15.18];
m(:,:,3)= [914.82 -21.86; 914.24 -26.48; 913.50 -25.16];
m(:,:,4)= [1101.83 -27.97; 1099.46 -30.05; 1100.02 -32.85];
m(:,:,5)= [1249.29 -176.12; 813.89 -611.09; 1245 -180.10];

gtruth= [500 0; 700 0; 900 0; 1100 0; 1300 0]; % for 1st transmitter
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