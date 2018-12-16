%% Rotation tests. 360, different radii % DATA=IN
%m_r= [80 78] % scratch this.. Rotation on its own axis

m_r=    [-30 17 17; % radius = 0 % error in degrees
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

%% Complex path
m_c= [47 45 41; % mm error % TEST 1 l:250 r:300
      24 30 30]; %degree error

m_c(:,:,2)= [35 32 20; % mm error % TEST 2 l:250 r:450
             36 30 43]; % degree error
    
m_c(:,:,3)= [47 105 105; % mm error & TEST 3 l:500 :r450
             36 28 31]; % Degree error 

m_c(:,:,4)= [570 385 410; % mm TEST 4: l:1000 r:450
             27 45 45]; %degree error %Note 
  
for i=1:size(m_c,3)
    for j=1:size(m_c,1)
        complexAcc(i,j)= sum(m_c(j,:,i))/size(m_c,2);
    end
end
complexAcc

for i=1:size(m_c,3)
    for j=1:size(m_c,1)
        complexVar(i,j)= var(m_c(j,:,i));
    end
end
complexStdv= sqrt(complexVar)

ranges= [2500,3300,3000,3000];