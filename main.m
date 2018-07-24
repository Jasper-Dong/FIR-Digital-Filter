[x,Fs]=audioread('o1.WAV');
x=x';
sound(x,Fs)
N=length(x);
n=0:N-1;
t=n/Fs;
s1=0.01*cos(2*pi*3000*Fs/N*t);
s2=0.03*randn(1,N);
figure ('name','单一频率噪声')
[y1,H1]=add_noise(x,s1,Fs,N);
sound(y1,Fs)
figure ('name','白噪声')
[y2,H2]=add_noise(x,s2,Fs,N);
sound(y2,Fs)
flp=1500;fls=1800;
fup=5000;fus=6000;
type='low';
figure ('name','汉宁窗函数滤波器特性')
[hn1,Hw1]=window_filter(fls,flp,fup,fus,type,Fs);
figure ('name','频率采样滤波器特性')
[hn2,Hw2]=sample_filter(fls,flp,fup,fus,type,Fs);
figure ('name','单频率噪声滤燥')
[z11,z12]=wipe_noise(y1,hn1,hn2,N,Fs);
figure ('name','白噪声滤燥')
[z21,z22]=wipe_noise(y2,hn1,hn2,N,Fs);
