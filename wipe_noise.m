function [z1,z2]=wipe_noise(y,hn1,hn2,N,Fs)
n=0:N-1;
t=n/Fs;
z1=fftfilt(hn1,y);
z2=fftfilt(hn2,y);
H3=fft(z1,N);
H4=fft(z2,N);
subplot(3,2,1)
plot(t,z1)
xlabel('t')
ylabel('z(t)')
title('窗函数滤波后信号')
subplot(3,2,2)
plot(t,z2)
xlabel('t')
ylabel('z(t)')
title('频率采样滤波后信号')
subplot(3,1,2)
plot(n*Fs/N,abs(H3))
xlabel('f')
ylabel('|Hg(\Omega)|')
title('窗函数滤波后信号频谱')
subplot(3,1,3)
plot(n*Fs/N,abs(H4))
xlabel('f')
ylabel('|Hg(\Omega)|')
title('频率采样滤波后信号频谱')


