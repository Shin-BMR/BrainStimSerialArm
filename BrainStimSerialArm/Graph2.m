close all
clear all 
clc

D2R=pi/180;
R2D=180/pi;

data_enc1 = dlmread('data_encoder1.txt','\t', 1, 0);
data_enc2 = dlmread('data_encoder2.txt','\t', 1, 0);
data_enc3 = dlmread('data_encoder3.txt','\t', 1, 0);
data_enc4 = dlmread('data_encoder4.txt','\t', 1, 0);
data_enc5 = dlmread('data_encoder5.txt','\t', 1, 0);
data_enc6 = dlmread('data_encoder6.txt','\t', 1, 0);

figure(1)
subplot(3,1,1);hold on;grid on;
plot(data_enc1(:,1),data_enc1(:,2));
%figure(2)
subplot(3,1,2);hold on;grid on;
plot(data_enc2(:,1),data_enc2(:,2));
%figure(3)
subplot(3,1,3);hold on;grid on;
plot(data_enc3(:,1),data_enc3(:,2));
figure(4)
subplot(3,1,1);hold on;grid on;
plot(data_enc4(:,1),data_enc4(:,2));
subplot(3,1,2);hold on;grid on;
%figure(5)
plot(data_enc5(:,1),data_enc5(:,2));
%figure(6)
subplot(3,1,3);hold on;grid on;
plot(data_enc6(:,1),data_enc6(:,2));