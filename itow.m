function[We]=itow(A,R,t,I,Wr)
%We:calculated worldcoordinate of this point
%Wr:3*1,real worldcoordinate of this point
%I:2*1,real imagecoordinate of this point
%A:3*3,intrinsic matrix
%R:3*3,rotation matrix
%t:3*1,translation matrix
i=A*(R*Wr+t);
%calculated imagecoordinate of this point
%according to real worldcoordinate,in order to get 'z'
z=i(3);
We=z*R'*inv(A)*[I;1]-R'*t;