function [hn,Hw]=sample_filter(fls,flp,fup,fus,type,Fs)
wp=2*pi*flp/Fs;
ws=2*pi*fls/Fs;
wup=2*pi*fup/Fs;
wus=2*pi*fus/Fs;
Bt=ws-wp;
m=1;
T=0.38;
N0=ceil((m+1)*2*pi/Bt);
N=N0+mod(N0+1,2);
n=0:N-1;
if strcmp(type,'low')  
    Np=fix(wp/(2*pi/N));
    Ns=N-2*Np
    Hk=[ones(1,Np),zeros(1,Ns),ones(1,Np)];
    Hk(Np+1)=T;
    Hk(N-Np)=T;
elseif strcmp(type,'high')
    Ns=fix(ws/(2*pi/N));
    Np=N-2*Ns;
    Hk=[zeros(1,Ns),ones(1,Np),zeros(1,Ns)];
    Hk(Ns+1)=T;
    Hk(N-Ns)=T;
elseif strcmp(type,'bandpass')
    Ns=fix(ws/(2*pi/N))
    Nup=fix(wup/(2*pi/N));
    Np=Nup-Ns;
    band=mod(N,2);
    Nus=(N-band)/2-Nup;
    Hk=[zeros(1,Ns),ones(1,Np),zeros(1,Nus+band),zeros(1,Nus),ones(1,Np),zeros(1,Ns),];
    Hk(Ns+1)=T;
    Hk(Np+Ns)=T;
    Hk(N-Ns)=T;
    Hk(N-Ns-Np)=T;
elseif strcmp(type,'stop')
    Np=fix(wp/(2*pi/N));
    Nus=fix(wus/(2*pi/N));
    Ns=Nus-Np;
    band=mod(N,2);
    Nup=(N-band)/2-Nus;
    Hk=[ones(1,Np),zeros(1,Ns),ones(1,Nup+band),ones(1,Nup),zeros(1,Ns),ones(1,Np),];
    Hk(Np+1)=T;
    Hk(Np+Ns)=T;
    Hk(N-Np)=T;
    Hk(N-Ns-Np)=T;
end
thetak=-pi*(N-1)*(0:N-1)/N;
Hdk=Hk.*exp(j*thetak);
hn=real(ifft(Hdk));
[Hw,w]=ft1(hn,n,2000);
subplot(3,1,1)
plot(2*n/N,Hk,2*n/N,Hk,'*')
axis([-0.1,2.1,-0.1,1.1])
xlabel('w/\pi')
ylabel('Hg(w)')
title('�����ͨ��������')
subplot(3,1,2)
stem(n,hn,'.')
xlabel('n')
ylabel('h(n)')
title('ϵͳ��λ������Ӧ')
subplot(3,2,5)
plot(w/pi,abs(Hw))
axis([0,1,-0.1,1.1])
xlabel('w/\pi')
ylabel('|H(w)|')
title('ϵͳƵ������')
subplot(3,2,6)
plot(w/pi,20*log10(abs(Hw)))
axis([0,1,min(20*log10(abs(Hw))),10])
xlabel('w/\pi')
ylabel('20lg|H(w)|')
title('ϵͳ˥������')