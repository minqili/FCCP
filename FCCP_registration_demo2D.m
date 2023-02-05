%============================================
% This demo shows 2D points registration results by the algorithm from the paper:
% "Fast non-rigid points registration with cluster correspondences projection".
% Verison: 1.0
% Date : 14/4/2019
% Author  : Minqi Li
% Tested on MATLAB 2018a
%============================================
clear;
clc;
close all;
addpath('./Data');
addpath('./results');
addpath(genpath('fccp'))
%%======================================================================
% load data
%----------Generate data----
Ti=1; Te=1;

for k1=Ti:5
    for k2=Te:100%20
%       tmp_name=['save_fish_outlier_',num2str(k1),'_',num2str(k2),'.mat'];
%       tmp_name=['save_fish_occlusion_',num2str(k1),'_',num2str(k2),'.mat'];        
%       tmp_name=['save_fish_noise_',num2str(k1),'_',num2str(k2),'.mat'];        
      tmp_name=['save_fish_def_',num2str(k1),'_',num2str(k2),'.mat'];
 
%       tmp_name=['save_chinese_def_',num2str(k1),'_',num2str(k2),'.mat'];    
%       tmp_name=['save_chinese_noise_',num2str(k1),'_',num2str(k2),'.mat'];         
%       tmp_name=['save_chinese_outlier_',num2str(k1),'_',num2str(k2),'.mat'];     
%       tmp_name=['save_chinese_occlusion_',num2str(k1),'_',num2str(k2),'.mat'];      
           
      load (tmp_name);
      X = x1; Y = y2a;  

    rand_correspondence=1
    if rand_correspondence==1
        Y0=Y;
        p = randperm(size(Y,1),size(Y,1));    
        Y = Y0(p,:);
    end

    add_noise=0
    if add_noise==1
        noise = sqrt(j-1)*randn(size(Y));  
        Y=Y+noise;
    end  

    figure(1), 
    cpd_plot_iter(X, Y);  title('Before registration'); 

    Dim=size(Y,2); 
    N=size(X,1);M=size(Y,1); 

    t1a=clock;
    opt.viz = 1;
    opt.outliers = 0
    opt.t = 1-  size(X,1)/size(Y,1);
    if opt.t>0
      opt.outliers = opt.t;  
    end
    opt.sparse = 1;
    opt.nsc = 5;

     [Transform, C]=fccp_register(Y, X, opt);
     Transform.X=Transform.Y;
     V = Transform.X;
     t1b=clock;   
     figure(2),cpd_plot_iter(Transform.X, Y); axis off; title('After registering Y to X');    
   end
end



