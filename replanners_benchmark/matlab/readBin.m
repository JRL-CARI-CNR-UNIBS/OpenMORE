clear all; close all; clc;

fileId = fopen('test_bin/test_q_0_i_0.bin');
fread(fileId,[7 8])

fileId = fopen('test_bin/test_q_0_i_0.bin');
success = fread(fileId,1)

number_of_objects    = fread(fileId,1,'uchar')
number_of_collisions = fread(fileId,1,'uchar')

fseek(fileId,6,'cof');
path_length = fread(fileId,1,'double')
distance_start_goal = fread(fileId,1,'double')
time = fread(fileId,1,'double')
replanning_time_mean = fread(fileId,1,'double')
replanning_time_std_dev = fread(fileId,1,'double')
