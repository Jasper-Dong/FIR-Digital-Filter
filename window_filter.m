function [hn,Hw]=window_filter(fls,flp,fup,fus,type,Fs)
wp=2*pi*flp/Fs;
ws=2*pi*fls/Fs;
wup=2*pi*fup/Fs;
wus=2*pi*fus/Fs;
Bt=abs(ws-wp);
N0=ceil(6.2*pi/Bt);
N=N0+mod(N0+1,2);
n=0:N-1;
if strcmp(type,'low') || strcmp(type,'high') 
            wc=(wp+ws)/2/pi;
end
if strcmp(type,'stop') || strcmp(type,'bandpass')
           wc=[(wp+ws)/2/pi,(wup+wus)/2/pi];
end
wn=hanning(N);
hn=fir1(N-1,wc,type,hanning(N));
[Hw,w]=ft1(hn,n,500);
subplot(3,1,1)
stem(n,hn,'.')
xlabel('n')
ylabel('h(n)')
title('系统单位脉冲响应')
subplot(3,1,2)
stem(n,wn,'.')
axis([min(n),max(n),-0.1,1.1])
xlabel('n')
ylabel('w(n)')
title('汉宁窗函数曲线')
subplot(3,2,5)
plot(w/pi,abs(Hw))
axis([0,1,-0.1,1.1])
xlabel('w/\pi')
ylabel('|H(w)|')
title('系统频谱曲线')
subplot(3,2,6)
plot(w/pi,20*log10(abs(Hw)))
axis([0,1,min(20*log10(abs(Hw))),10])
xlabel('w/\pi')
ylabel('20lg|H(w)|')
title('系统衰减函数')