clear;
clc;

site_name = 'http://192.168.4.1/';
options = weboptions;
options.Timeout = 15;

data = struct();
% percent weight for weighted average
data.lidar_weight = 0;     
data.smoothing = 'weighted_average';

% data.smoothing = 'var_average';
% data.lidar_var = 10;
% data.imu_var = 0.1;

site = [site_name, 'data_processing']; 

response = webwrite(site, data, options);


