%% Recorded positions % DATA = IN % Transmitter 4 facing marks
m_0= [0 0; 0 1; -1 1];

m= [5.50 457.61; 9.18 455.77; 12.85 455.94];
m(:,:,2)= [18.42 646.84; 20.33 640.64; 18.42 637.56];
m(:,:,3)= [11.94 904.40; 11.94 906.88; 9.95 908.69];
m(:,:,4)= [3.49 1113.15; 3.49 1113.85; 3.49 1113.15];
m(:,:,5)= [0.73 1323.27; -0.73 1327.25; -2.96 1333.45];

gtruth= [0 500; 0 700; 0 900; 0 1100; 0 1300];
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