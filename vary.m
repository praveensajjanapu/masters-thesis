
clc;
close all;
clear all;

PbdB = 0:2:20;
PrdB = 10;
gamThdB = 2;
N0 = 1;
NOL = 10^5;
%%-------------------------------------------------------------------------------------------------------------------
NB=2;
NR1=2;
NR2=2;
NU=2;

NB_2=3;
NR1_2=3;
NR2_2=3;
NU_2=3;

NB_3=4;
NR1_3=4;
NR2_3=4;
NU_3=4;
%%----------------------------------------channels NOMA with 2 UEs------------------------------------------------------------
n = 3;% Pathloss exponent
% Distance from BS to Relay 1 and 2 
d1 = [0.5 2];
Omega1 = d1.^(-n);
% Distance from Relay 1 and 2 to User 1 and 2
d2 = [0.7 0.7]; 
Omega2 = d2.^(-n);
m1 = [2 2];
m2 = [2 2];
%%----------------------------------------Modulation parameters------------------------------------------------------------
M=8; % MPSK
a_MPSK=2;
b_MPSK=2*(sin(pi/M)^2);
%%-------------------------------------------------------------------------------------------------------------------
[OP,SER,CAP] = OP_SER_CAP(m1,m2,Omega1,Omega2,PbdB,PrdB,gamThdB,N0,NOL,NB,NR1,NR2,NU,a_MPSK,b_MPSK);
[OP1,SER1,CAP1] = OP_SER_CAP(m1,m2,Omega1,Omega2,PbdB,PrdB,gamThdB,N0,NOL,NB_2,NR1_2,NR2_2,NU_2,a_MPSK,b_MPSK);
[OP2,SER2,CAP2] = OP_SER_CAP(m1,m2,Omega1,Omega2,PbdB,PrdB,gamThdB,N0,NOL,NB_3,NR1_3,NR2_3,NU_3,a_MPSK,b_MPSK);


figure(1);
semilogy(PbdB, OP(:,1).*OP(:,2),'-*','LineWidth',2);
hold on
semilogy(PbdB, OP1(:,1).*OP1(:,2),'-o','LineWidth',2);
hold on
semilogy(PbdB, OP2(:,1).*OP2(:,2),'-h','LineWidth',2);
ylabel('Outage Probability'); xlabel('Transmit SNR at The Base Station (dB)');
legend('NOMA with 2 antennas at all terminals','NOMA with 3 antennas at all terminals','NOMA with 4 antennas at all terminals ');
%axis([10^7 1*10^11 10^-6 1]);
axis([0 20 10^-5 1]);
grid on

figure(2);
semilogy(PbdB, (SER(:,1)+SER(:,2))/2,'-*','LineWidth',2);
hold on
semilogy(PbdB, (SER1(:,1)+SER1(:,2))/2,'-o','LineWidth',2);
hold on
semilogy(PbdB, (SER2(:,1)+SER2(:,2))/2,'-h','LineWidth',2);
ylabel('Symbol Error Rate'); xlabel('Transmit SNR at The Base Station (dB)');
legend('NOMA with 2 antennas at all terminals','NOMA with 3 antennas at all terminals','NOMA with 4 antennas at all terminals');
%axis([10^7 1*10^11 10^-6 1]);
grid on

figure(3);
plot(PbdB,  CAP(:,1)+CAP(:,2),'-*','LineWidth',2);
hold on
plot(PbdB,  CAP1(:,1)+CAP1(:,2),'-o','LineWidth',2);
hold on
plot(PbdB, CAP2(:,1)+CAP2(:,2),'-h','LineWidth',2);
ylabel('Capacity (bps/Hz)'); xlabel('Transmit SNR at The Base Station (dB)');
legend('NOMA with 2 antennas at all terminals','NOMA with 3 antennas at all terminals','NOMA with 4 antennas at all terminals');
%axis([10^7 1*10^11 10^-6 1]);
grid on
% dlmwrite('OP_Freq_lamda_05_P5.txt',OP_Freq_lamda_05_P5','precision', '%.20f','newline','pc');
% dlmwrite('OP_Freq_lamda_15_P5.txt',OP_Freq_lamda_15_P5','precision', '%.20f','newline','pc');
% dlmwrite('OP_Freq_lamda_25_P5.txt',OP_Freq_lamda_25_P5','precision', '%.20f','newline','pc');
% dlmwrite('OP_Freq_lamda_35_P5.txt',OP_Freq_lamda_35_P5','precision', '%.20f','newline','pc');
% dlmwrite('OP_Freq_lamda_45_P5.txt',OP_Freq_lamda_45_P5','precision', '%.20f','newline','pc');
% 
