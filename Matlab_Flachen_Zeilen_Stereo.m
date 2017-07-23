%clear all;
close all;
clc;

%Flachen:   

Error=cameraParams.ReprojectionErrors;
for i=1:25
    E(:,:,i)=Error(:,:,i).^2;
    N=sum(E(:,:,i));
    M(i)=sum(N(:));
end

[N,n]=min(M);   %n position of mini Error

Rl=cameraParams.RotationMatrices;
Rl=Rl(:,:,n)';
Tl=cameraParams.TranslationVectors;
Tl=Tl(n,:)';
AF=cameraParams.IntrinsicMatrix;
AF=AF';
RL=[Rl,Tl;0, 0, 0, 1];

I_f1=imread('calib_area.bmp');% name of choosed area-scan camera picture
[imagePoints_f1,boardSize1]=detectCheckerboardPoints(I_f1);
imagepos_f(:,:,1)=imagePoints_f1;

IF1=imagePoints_f1(13,:)';   %Image position 1
IF2=imagePoints_f1(14,:)';   %Image position 2
WrF=cameraParams.WorldPoints;
WrF1=[WrF(13,:)';0];
WrF2=[WrF(14,:)';0];   %World positon 2

%{
Rl=[0.0048 -1 0.0087;0.9998 0.0046 -0.0170;0.0169 0.0087 0.9998];   %RotationM
Tl=[37.2566 -47.3591 497.9225]';    %TranslationV
AF=[6974.6 0 983.1057;0 6953.4 1001.3;0 0 1];   %IntrinsicMatrix
RL=[Rl,Tl;0, 0, 0, 1];


IF1=[1506 1178]';   %Image position 1
WrF1=[60 0 0]';    %World positon 1
IF2=[1227 1179]'; %Image position 2
WrF2=[41.1 13.8 0]';  %World positon 2
%}

S1x=itow(AF,Rl,Tl,IF1,WrF1);    %Calculate positon 1
S1x=S1x(1);
S1y=itow(AF,Rl,Tl,IF1,WrF1);
S1y=S1y(2);

S2x=itow(AF,Rl,Tl,IF2,WrF2);    %Calculate positon 2
S2x=S2x(1);
S2y=itow(AF,Rl,Tl,IF2,WrF2);
S2y=S2y(2);

%Zeilen

H = imread('c-z5.png');  %name of line-scan picture
I=rgb2gray(H);
plot(I);    %grayscale
title('Line data grey scale distribution');
xlabel('X-axis pixel location');
ylabel('Grayscale');
k=60;    %line-scan picture threshold value
n = 1;
m = length(I);
for a = 1:m 
   if I(a)>k;
      I(a)=0;
   end;
end;
b=1;
s=0;
for c=1:m
     if s==0 && I(c)>0
      pos(b)=c;
      b=b+1;
      s=1;
     end;

     if I(c)>0 && I(c+1)==0 && s==1
      pos(b)=c;
      b=b+1;
      s=0;
     end;
end;
d=1;
m = length(pos);
while d <= m
   num = (d+1)/2;
   cp(num) = round((pos(d)+pos(d+1))/2);
   d = d+2;
end;

imagepos1=cp;  %cp have not given
worldpos1=zeros(11,2);
h=100;      %high of zikzak, have not given
d=20;       %distance of zikzak, have not given

for i=1:5
    x=d*((cp(2*i)-cp(2*i-1))/(cp(2*i+1)-cp(2*i-1)))+(i-1)*d;
    y=h*((cp(2*i)-cp(2*i-1))/(cp(2*i+1)-cp(2*i-1)));
    worldpos1(2*i,:)=[x,y];
end

worldpos1(1,:)=[0,worldpos1(2,2)-worldpos1(2,1)*(worldpos1(4,2)-worldpos1(2,2))/(worldpos1(4,1)-worldpos1(2,1))];
%first point

for i=1:4
    x=i*d;%here x,y can be used again or not?
    y=worldpos1(2*i,2)+(x-worldpos1(2*i,1))*(worldpos1(2*i+2,2)-worldpos1(2*i,2))/(worldpos1(2*i+2,1)-worldpos1(2*i,1));
    worldpos1(2*i+1,:)=[x,y];
end

worldpos1(11,:)=[5*d,worldpos1(10,2)+(5*d-worldpos1(10,1))*(worldpos1(10,2)-worldpos1(8,2))/(worldpos1(10,1)-worldpos1(8,1))];
%last point

cx=375;% principle point, half of number of pixel, have not given
cy=375;% principle point, half of number of pixel, have not given

while 1

    i=i+1;
   
    imagepos2=[imagepos1',cy*ones(11,1);cx*ones(11,1),imagepos1';];
    
    tx=d*(cx-cp(1))/(cp(3)-cp(1));  % torsion center
    ty=worldpos1(1,2)+tx/d*(worldpos1(3,2)-worldpos1(1,2)); % torsion center
    worldpos2=[worldpos1;(tx-ty)*ones(11,1)+worldpos1(:,2),(tx+ty)*ones(11,1)-worldpos1(:,1)];
    A=[0*ones(5,1);1*ones(6,1);0*ones(5,1);1*ones(6,1)];
    worldpos3=[worldpos2 A];
    
    [A R t]=tsai([worldpos3 imagepos2]);
    u0=A(1,3);
    v0=A(2,3);   
    
    error=sqrt((u0-cx)^2+(v0-cy)^2);
   
    if error<1 % 10 can be changed     
        break;
    else
        cx=u0;
        cy=v0;
    end
end

cx  %principle point x
cy  %principle point y

expectedpos=map3d (A,R,t,worldpos3')';
rep_error=sum(sqrt(sum((imagepos2-expectedpos).^2,2)))/length(worldpos2)

Rr=R; %RotationM
Tr=t;   %TranslationV
AZ=A; %IntrinsicMatrix
RR=[Rr,Tr;0, 0, 0, 1];

IZ1=imagepos2(3,:)';   %Image position 1
WrZ1= [worldpos1(3,:)';0];   %World positon 1
IZ2=imagepos2(5,:)';    %Image position 2
WrZ2= [worldpos1(5,:)';0];   %World positon 2

Z1x=itow(AZ,Rr,Tr,IZ1,WrZ1);    %Calculate positon 1
Z1x=Z1x(1);
Z1y=itow(AZ,Rr,Tr,IZ1,WrZ1);
Z1y=Z1y(2);

Z2x=itow(AZ,Rr,Tr,IZ2,WrZ2);    %Calculate positon 2
Z2x=Z2x(1);
Z2y=itow(AZ,Rr,Tr,IZ2,WrZ2);
Z2y=Z2y(2);

%zeilen imageCS to Flachen imageCS
Rww=[1 0 0; 0 -1 0; 0 0 -1]*[0 1 0; -1 0 0; 0 0 1]; %rotation from zeilen worldCS to Flachen worldCS
Tww=[200 -20 0]'; %translation from zeilen WCS to Flachen WCS
Rw=[Rww,Tww;0, 0, 0, 1];


Rfw=Rw*RR^(-1);  %zeilen imageCS to Flachen worldCS

R=RL*Rfw    %zeilen imageCS to Flachen imageCS

distanceZF=((R(1,4))^2+(R(2,4))^2+(R(3,4))^2)^0.5

%Evalution
TZ1x=Z1y+200;   %Zeilen coordinate system change
TZ1y=Z1x-20;
TZ2x=Z2y+200;
TZ2y=Z2x-20;

distanceX1=TZ1x-S1x;    

distanceY1=TZ1y-S1y;

distanceX2=TZ2x-S2x;

distanceY2=TZ2y-S2y;

d=1;    %acceptable error
length=120; %Distance between two images
unit=20;

d1=distanceX1-Z1y;  %actual (length+unit)
if abs(d1-length-unit)<d
    delta_d1=abs(d1-length-unit)
    disp('Acceptable');    
end

d2=distanceX2-Z2y;
if abs(d2-length-unit)<d
    delta_d2=abs(d2-length-unit)
    disp('Acceptable');    
end



    
