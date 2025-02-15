clc;
close all;
clear all;


PbdB = 0:2:20;
PrdB = 10;
gamThdB = 2;
N0 = 1;
NOL = 10^5;
Pb_OMA = (10.^(PbdB/10))/2;
Pr_OMA = (10.^(PrdB/10))/2;
Pb_OMA_dB = 10*log10(Pb_OMA);
Pr_OMA_dB = 10*log10(Pr_OMA);

%%-------------------------------------------------------------------------------------------------------------------
NB=2;
NR1=2;
NR2=2;
NU=2;
%%----------------------------------------channels NOMA with 2 UEs------------------------------------------------------------
n = 3;% Pathloss exponent
% Distance from BS to Relay 1 and 2 
d1 = [0.5 2];
Omega1 = d1.^(-n);
%one user
d_1 = [1.2];
Omega_1 = d_1.^(-n);
% Distance from Relay 1 and 2 to User 1 and 2
d2 = [0.7 0.7]; 
Omega2 = d2.^(-n);

d_2 = [0.7]; 
Omega_2 = d_2.^(-n);

m1 = [2 2];
m2 = [2 2];

m_1 = 2;
m_2 = 2;
%%----------------------------------------Modulation parameters------------------------------------------------------------
M=8; % MPSK
a_MPSK=2;
b_MPSK=2*(sin(pi/M)^2);
%%-------------------------------------------------------------------------------------------------------------------
[OP,SER,CAP] = OP_SER_CAP(m1,m2,Omega1,Omega2,PbdB,PrdB,gamThdB,N0,NOL,NB,NR1,NR2,NU,a_MPSK,b_MPSK);
[OP1,SER1,CAP1] = OP_SER_CAP(m_1,m_2,Omega_1,Omega_2,Pb_OMA_dB,Pr_OMA_dB,gamThdB,N0,NOL,NB,NR1,NR2,NU,a_MPSK,b_MPSK);

figure(1);
semilogy(PbdB, OP(:,1).*OP(:,2),'-o','LineWidth',2);
hold on
semilogy(PbdB, OP1(:,1),'-h','LineWidth',2);
ylabel('Outage Probability'); xlabel('Transmit SNR at The Base Station (dB)');
legend('2 Users NOMA','1 User OMA');
axis([0 20 10^-5 1]);
grid on

figure(2);
semilogy(PbdB, (SER(:,1)+ SER(:,2))/2,'-o','LineWidth',2);
hold on
semilogy(PbdB, SER1(:,1),'-h','LineWidth',2);
ylabel('Symbol Error Rate'); xlabel('Transmit SNR at The Base Station (dB)');
legend('2 Users NOMA','1 User OMA');
%axis([10^7 1*10^11 10^-6 1]);
%axis([0 20 10^-6 1]);
grid on

figure(3);
plot(PbdB, CAP(:,1)+CAP(:,2),'-o','LineWidth',2);
hold on
plot(PbdB, CAP1(:,1),'-h','LineWidth',2);
ylabel('Capacity (bps/Hz)'); xlabel('Transmit SNR at The Base Station (dB)');
legend('2 Users NOMA','1 User OMA');
%axis([10^7 1*10^11 10^-6 1]);
%axis([0 20 10^-6 1]);
grid on
% dlmwrite('OP_Freq_lamda_05_P5.txt',OP_Freq_lamda_05_P5','precision', '%.20f','newline','pc');
% dlmwrite('OP_Freq_lamda_15_P5.txt',OP_Freq_lamda_15_P5','precision', '%.20f','newline','pc');
% dlmwrite('OP_Freq_lamda_25_P5.txt',OP_Freq_lamda_25_P5','precision', '%.20f','newline','pc');
% dlmwrite('OP_Freq_lamda_35_P5.txt',OP_Freq_lamda_35_P5','precision', '%.20f','newline','pc');
% dlmwrite('OP_Freq_lamda_45_P5.txt',OP_Freq_lamda_45_P5','precision', '%.20f','newline','pc');
% 
