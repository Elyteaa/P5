%%Recorded data and grount truth % DATA = MISSING
m = [91 91 91; 115 114 116; 250 252 247];

%m = []; %Other part of the test
gtruth = [90 115 250];


%% Number scronching
avg= sum(m,2)/size(m,2); %Vector of averages
accVec= zeros([size(m,2) 1]);
accVec(:,:)=[];

for i=1:size(avg)
    accVec(i)= abs(avg(i)-gtruth(i));
end

accVec

varVec= zeros([size(m,2) 1]);
varVec(:,:)=[];

for i=1:size(m,2)
    varVec(i)= var(m(i,:));
end

varVec