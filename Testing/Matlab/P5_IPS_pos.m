%% Recorded positions % DATA = MISSING
m_0= [0 0; 0 1; -1 1];

m= [500 0; 497 0; 505 0];
m(:,:,2)= [700 0; 699 0; 703 0];
m(:,:,3)= [900 0; 889 0; 910 0];

gtruth= [500 0; 700 0; 900 0];

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