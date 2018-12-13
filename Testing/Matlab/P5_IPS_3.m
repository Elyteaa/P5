%% Input recorded distances from transmitters, 3rd round of testing % DATA = MISSING
m=    [1662 1661 1661; %Position 1, 1st transmitter
       1745 1745 1745; %Position 1, 2nd transmitter
       1892 1894 1894;
       1745 1745 1744];
    
m(:,:,2)=  [1624 1623 1624; %Position 2, 1st transmitter
            1812 1811 1812;
            2003 2005 2005;
            1808 1809 1809];
    
m(:,:,3)=  [1658 1658 1659; %...
            1898 1897 1896;
            2133 2133 2132;
            1899 1899 1899];

m(:,:,4)=  [1726 1728 1728;
            2003 2006 2005;
            2274 2273 2276;
            1999 2000 1998];

%m(:,:,5)= [1839; 
%           2120;
%           2470;
%           ????] %transmitter 4 isn't giving anything here

%% Calculated individual distances, based on our measurements
%   Used as ground truth
d_up= 1620;     %Distance from floor to sensors
d_arm= 505;     %Distance from top center of tower to transmitter
d_p= 200;       %Distance between points
% 1580, 480, 200 %suggestion 1
% 1620, 505, 200

gtruth= zeros([size(m,1),size(m,3)]); %Nr of trans, nr of pos
gtruth(:,:)=[];

for i=1:size(m,1) %Transmitter index
    for j=1:size(m,3) %Position index
        if i == 1
            gtruth(i,j)= sqrt(d_up^2 + ((j-1)*d_p)^2);
        elseif i == 2 || i == 4
            gtruth(i,j)= sqrt(d_up^2 + ((d_arm+(j-1)*d_p)^2)+d_arm^2);
        elseif i == 3
            gtruth(i,j)= sqrt(d_up^2 + (2*d_arm+(j-1)*d_p)^2);
        end
    end
 end
gtruth %(trans, pos)

%% Number crunching
avg= sum(m,2)/size(m,2); %sum all measurements of a transmitter, divide by number of measurements
accmat= zeros([size(m,1),size(m,3)]);
accmat(:,:)=[];

for i=1:size(m,3) %Position index
    for j=1:size(m,1) %Transmitter index
        accmat(j,i)= abs(avg(j,1,i)-gtruth(j,i));
    end
end

accmat %(trans, pos)

varmat= zeros([size(m,1),size(m,3)]);
varmat(:,:)=[];

for i=1:size(m,3) %Position index
    for j=1:size(m,1) %Sensor index
        varmat(j,i)= var(m(j,:,i));
    end
end

varmat %(trans, pos)

stdvmat= sqrt(varmat) %(trans, pos)

%Average standard deviation
avgStdv= sum(sum(stdvmat))/(size(m,1)*size(m,3))

%Average accuracy
avgAcc= sum(sum(accmat))/(size(m,1)*size(m,3))