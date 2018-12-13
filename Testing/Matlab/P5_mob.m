%% Distance measurements for the linear movement
%m_1= [15 14 13; 23 20 22; 42 37 42]; %Simple distances?

%% Rotation tests. 360, different radii % DATA=MISSING
%m_r= [80 78] % scratch this.. Rotation on its own axis

m_r=    [30 17 17; % radius = 0 % error in degrees
         40 17 24; % radius = 300mm
         10 40 23];% radius = 500mm

avgdegErr= zeros([3 1]);
avgdegErr(:,:)=[];

for i=1:size(m_r,1)
    avgdegErr(i,1)= sum(m_r(i,:))/size(m_r,1);
end
avgdegErr

%% Linear movements, recording the error. [x y] x=straight ahead, y=right %DATA=IN
m_l= [-15 5; -11 9; -12 3]; %1st distance: 250 mm
m_l(:,:,2)= [-20 10; -19 2; -20 8]; % 500 mm
m_l(:,:,3)= [-38 28; -37 20; -38 27]; % 1000 mm
pathlen= [250 500 1000]';

distErr= zeros([size(m_l,1),size(m_l,3)]);
distErr(:,:)=[];

for i=1:size(m_l,3) %Length index
    for j=1:size(m_l,3) %Attempt index
        distErr(j,i)= sqrt((m_l(j,1,i))^2 + (m_l(j,2,i))^2);
    end
end
distErr

avgdistErr= zeros([size(distErr,1) 1]);
avgdistErr(:,:)=[];
for i=1:size(distErr,1)
    avgdistErr(i,1)= sum(distErr(i,:))/size(distErr,2);
end
avgdistErr

reldistErr= zeros([size(m_l,1), 1]);
reldistErr(:,:)=[];
for i=1:size(distErr,1)
    reldistErr(i,1)= 100*avgdistErr(i,1)/pathlen(i,1);
end
reldistErr

avgErr= zeros([2 1]);
avgErr(:,:)=[];

avgErr(1,1)= sum(sum(m_l(:,1,:)))/(size(m_l,1)*size(m_l,3));
avgErr(2,1)= sum(sum(m_l(:,2,:)))/(size(m_l,1)*size(m_l,3))