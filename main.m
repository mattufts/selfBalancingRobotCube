clear;
clc;

site_name = 'http://192.168.4.1/';
options = weboptions;
options.Timeout = 15;

data = struct();
site = [site_name, 'sendangles']; 

response = webwrite(site, data, options);


