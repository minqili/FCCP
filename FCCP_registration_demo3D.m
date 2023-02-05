%============================================
% This demo shows 3D points registration results by the algorithm from the paper:
% "Fast non-rigid points registration with cluster correspondences projection".
% Verison: 1.0
% Date : 14/4/2019
% Author : Minqi Li
% Tested on MATLAB 2018a
%============================================
clear; clc; close all;
addpath('./3Ddata');
addpath('./results');
addpath(genpath('fccp'));
addpath (genpath('Point_cloud_tools_for_Matlab-master'));

%=============================================
%load data for registration 
%load_data: 1 deformable 3d data; 2 Standford_Bunny data
load_data=1;  
if load_data==1  %deformable 3d data 
%     fname_X='horse-gallop-35';
%     fname_Y='horse-gallop-36';
    fname_Y='lion-06'; 
    fname_X='lion-reference';
    
    pcScan1 = pointCloud([fname_X, '.ply']);
    rate=round(size(pcScan1.X)/10000);
    pcScan1.select('IntervalSampling',rate(1));
    
    X=pcScan1.X(find(pcScan1.act>0),:);
    pcshow2(X(:,1:3),[0,0,1]);   %blue  
    
    hold on; 
    pcScan2 = pointCloud([fname_Y,'.ply']);
    pcScan2.select('IntervalSampling', rate(1));
    Y=pcScan2.X(find(pcScan2.act>0),:);
    
    pcshow2(Y(:,1:3),[1,0,0]);
    view(2);
    grid off;
    axis off;
    title('Before registration');
end

if load_data==2  %Standford_Bunny data
    load bunny.mat;
    load bunny_ref.mat;
         
    add_outliers=0
    if add_outliers==1
    Y0=Y;
    Y3_max=max(Y);  
    Y3_min=min(Y);
    n0=round(0.1*size(Y,1));    
    outliers3d=repmat(mean(Y0),n0,1)+[(Y3_max(1)-Y3_min(1))*0.25*randn(n0,1),(Y3_max(2)-Y3_min(2))*0.25*randn(n0,1),(Y3_max(3)-Y3_min(3))*0.25*randn(n0,1)];
    Y=[Y;outliers3d];
    end

    % plot the points cloud
    figure;
    view=[0,0,800,800];MarkerSize=8;  
    my_plot3d(X,Y,view,2,MarkerSize);
    axis off;
    axis equal;
    grid off;
    title('Before registration'); 
end

Dim=size(Y,2); 
N=size(X,1);M=size(Y,1); 

t1a=clock;
opt.outliers = 0;
opt.viz = 0;
opt.t = 1-  size(X,1)/size(Y,1);
opt.sparse = 1;
opt.nsc = 5;

[Transform, C]=fccp_register(Y, X, opt);

Transform.X=Transform.Y;
V = Transform.X;
t1b=clock;

figure;
view=[0,0,800,800];MarkerSize=8;  
my_plot3d(Transform.X,Y,view,2,MarkerSize);
axis off;
axis equal;
grid off;
title('After registration'); 

errors1= ModHausdorffDist(Transform.X,Y)
time1=etime(t1b,t1a)




