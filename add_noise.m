function [y,H2]=add_noise(x,s,Fs,N)
n=0:N-1;
t=n/Fs;
H1=fft(x,N);
subplot(3,2,1)
plot(t,x)
xlabel('t')
ylabel('x(t)')
title('ԭ�ź�')
subplot(3,1,2)
plot(n*Fs/N,abs(H1))
xlabel('f')
ylabel('|H(\Omega)|')
title('ԭ�ź�Ƶ��')
y=s+x;
H2=fft(y,N);
subplot(3,2,2)
plot(t,y)
xlabel('t')
ylabel('x(t)+s(t)')
title('�������ź�')
subplot(3,1,3)
plot(n*Fs/N,abs(H2))
xlabel('f')
ylabel('|Hs(\Omega)|')
title('�������ź�Ƶ��')