clc; clear all; close all;

addpath("data\FAN_sound_error\");
addpath("data\FAN_sound_OK\");
N_sound_error = 13;
error_sound_filename = "FAN_sound_error_0%d.wav";
for idx = 1:9
    filename = sprintf(error_sound_filename,idx);
    [y,Fs] = audioread(filename);
    sound_error{idx,1} = y;
    sound_error{idx,2} = Fs;
end

error_sound_filename = "FAN_sound_error_%d.wav";

for idx = 10:13
    filename = sprintf(error_sound_filename,idx);
    [y,Fs] = audioread(filename);
    sound_error{idx,1} = y;
    sound_error{idx,2} = Fs;
end

