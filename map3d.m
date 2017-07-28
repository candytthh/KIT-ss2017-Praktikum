function [ I ] = map3d( A, R, t, W )
%MAP3D implements pinhole camera mapping with camera matrix A, and
%extrinsic parameters R (rotation matrix) and t (translation vector)
% I = map3d( A, R, t, W )
% W: 3xN - matrix, each column is one world coordinate
% I: 2xN - matrix, each column is one image coordinate
[d c]=size(W);
M=A*([R t]); % matrix M, see slides
for i=1:c % apply to every point:
xyz1 = [W(:,i); 1]; % extend vector by 1 (homogeneous coordinates)
xyz = M*xyz1;
I(1,i)=xyz(1)/xyz(3); % devide by z
I(2,i)=xyz(2)/xyz(3);
end