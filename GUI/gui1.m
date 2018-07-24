function varargout = gui1(varargin)
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @gui1_OpeningFcn, ...
                   'gui_OutputFcn',  @gui1_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end

%读取音频文件
function pushbutton3_Callback(hObject, eventdata, handles) 
global x Fs;
[FileName,PathName]=uigetfile('*.WAV');
voicefile=strcat(PathName,FileName);
[x,Fs]=audioread(voicefile);

%显示波形
function pushbutton8_Callback(hObject, eventdata, handles)
global x t Fs n N;
set(handles.pushbutton4,'Enable','off');
set(handles.pushbutton5,'Enable','off');
set(handles.pushbutton6,'Enable','off');
sound(x,Fs)
x=x';
sound(x,Fs);
N=length(x);
n=0:N-1;
t=n/Fs;
H1=fft(x,N);
axes(handles.axes1)
plot(t,x)
xlabel('t')
ylabel('x(t)')
title('原信号')
axes(handles.axes5)
plot(n*Fs/N,abs(H1))
xlabel('f')
ylabel('|H(\Omega)|')
title('原信号频谱')

%选择噪声种类
function popupmenu3_Callback(hObject, eventdata, handles)
global val
val=get(handles.popupmenu3,'Value');

switch val
    case 1
        set(handles.edit13,'Enable','on')
        
        set(handles.pushbutton4,'Enable','on');
    case 2
        set(handles.edit13,'Enable','off')
        
        set(handles.pushbutton4,'Enable','on');
    case 3
        
        set(handles.pushbutton4,'Enable','off');
end

%显示加入噪声后波形
function pushbutton4_Callback(hObject, eventdata, handles)
global t n N Fs x y val;
a=get(handles.edit12,'String');
f=get(handles.edit13,'String');
a=str2num(a)
f=str2num(f)
switch val
    case 1
        s=a*sin(2*0.75*pi*f*Fs/N*t);
    case 2
        s=a*randn(1,N);
    case 3
        s=zeros(1,N);
end
y=x+s;
sound(y,Fs)
H2=fft(y,N);
axes(handles.axes2)
plot(t,y)
xlabel('t')
ylabel('x(t)+s(t)')
title('带噪声信号')
axes(handles.axes6)
plot(n*Fs/N,abs(H2))
xlabel('f')
ylabel('|Hs(\Omega)|')
title('带噪声信号频谱')

%选择滤波器种类
function popupmenu6_Callback(hObject, eventdata, handles)
global type
val=get(handles.popupmenu6,'Value');
switch val
    case 5
        set(handles.edit8,'Enable','off');
        set(handles.edit9,'Enable','off');
        set(handles.edit10,'Enable','off');
        set(handles.edit11,'Enable','off');
    case 4
        type='low'
        set(handles.edit8,'Enable','on');
        set(handles.edit8,'String','通带截止频率');
        set(handles.edit9,'Enable','on');
        set(handles.edit9,'String','阻带截止频率');
        set(handles.edit10,'Enable','off');
        set(handles.edit11,'Enable','off');
    case 3
        type='high'
        set(handles.edit8,'Enable','on');
        set(handles.edit8,'String','通带截止频率');
        set(handles.edit9,'Enable','on');
        set(handles.edit9,'String','阻带截止频率');
        set(handles.edit10,'Enable','off');
        set(handles.edit11,'Enable','off');
    case 2
        type='stop'
        set(handles.edit8,'Enable','on');
        set(handles.edit8,'String','通带下截止频率');
        set(handles.edit9,'Enable','on');
        set(handles.edit9,'String','阻带下截止频率');
        set(handles.edit10,'Enable','on');
        set(handles.edit10,'String','通带上截止频率');
        set(handles.edit11,'Enable','on');
        set(handles.edit11,'String','阻带上截止频率');  
     case 1
         type='bandpass'
        set(handles.edit8,'Enable','on');
        set(handles.edit8,'String','通带下截止频率');
        set(handles.edit9,'Enable','on');
        set(handles.edit9,'String','阻带下截止频率');
        set(handles.edit10,'Enable','on');
        set(handles.edit10,'String','通带上截止频率');
        set(handles.edit11,'Enable','on');
        set(handles.edit11,'String','阻带上截止频率'); 
end


%选择滤波器方法
function popupmenu5_Callback(hObject, eventdata, handles)
global wp ws wup wus nf hn w Hw judge wn Hk  Nf type
Bt=abs(ws-wp);
m=1;
T=0.38;
val=get(handles.popupmenu5,'Value');
switch val
    case 1 
       set(handles.pushbutton6,'Enable','on');
       judge=1;
       N0=ceil(6.2*pi/Bt);
       Nf=N0+mod(N0+1,2);
       nf=0:Nf-1;
       if strcmp(type,'low') || strcmp(type,'high') 
            wc=(wp+ws)/2/pi;
       end
       if strcmp(type,'stop') || strcmp(type,'bandpass') 
           wc=[(wp+ws)/2/pi,(wup+wus)/2/pi];
       end
       wn=hanning(Nf);
       hn=fir1(Nf-1,wc,type,hanning(Nf));
       [Hw,w]=ft1(hn,nf,500);
    case 2
       set(handles.pushbutton6,'Enable','on');
       judge=2;
       N0=ceil((m+1)*2*pi/Bt);
       Nf=N0+mod(N0+1,2);
       nf=0:Nf-1;
       if strcmp(type,'low')
            Np=fix(wp/(2*pi/Nf));
            Ns=Nf-2*Np;
            Hk=[ones(1,Np),zeros(1,Ns),ones(1,Np)];
            Hk(Np+1)=T;
            Hk(Nf-Np)=T;
       elseif strcmp(type,'high')
            Ns=fix(ws/(2*pi/Nf));
            Np=Nf-2*Ns;
            Hk=[zeros(1,Ns),ones(1,Np),zeros(1,Ns)];
            Hk(Ns+1)=T;
            Hk(Nf-Ns)=T;
       elseif strcmp(type,'bandpass')
            Ns=fix(ws/(2*pi/Nf));
            Nup=fix(wup/(2*pi/Nf));
            Np=Nup-Ns;
            band=mod(Nf,2);
            Nus=(Nf-band)/2-Nup;
            Hk=[zeros(1,Ns),ones(1,Np),zeros(1,Nus+band),zeros(1,Nus),ones(1,Np),zeros(1,Ns),];
            Hk(Ns+1)=T;
            Hk(Np+Ns)=T;
            Hk(Nf-Ns)=T;
            Hk(Nf-Ns-Np)=T;
       elseif strcmp(type,'stop')
            Np=fix(wp/(2*pi/Nf));
            Nus=fix(wus/(2*pi/Nf));
            Ns=Nus-Np;
            band=mod(Nf,2);
            Nup=(Nf-band)/2-Nus;
            Hk=[ones(1,Np),zeros(1,Ns),ones(1,Nup+band),ones(1,Nup),zeros(1,Ns),ones(1,Np),];
            Hk(Np+1)=T;
            Hk(Np+Ns)=T;
            Hk(Nf-Np)=T;
            Hk(Nf-Ns-Np)=T;
       end
       thetak=-pi*(Nf-1)*(0:Nf-1)/Nf;
       Hdk=Hk.*exp(j*thetak);
       hn=real(ifft(Hdk));
       [Hw,w]=ft1(hn,nf,2000);
    case 3
        set(handles.pushbutton6,'Enable','off');
        hn=zeros(1,100);
        Hw=hn;
        w=100;
end


%显示滤波器特性
function pushbutton6_Callback(hObject, eventdata, handles)
global nf w Hw judge wn Hk Nf
axes(handles.axes9)
if judge==1
    stem(nf,wn,'.')
    axis([min(nf),max(nf),-0.1,1.1])
    xlabel('n')
    ylabel('w(n)')
    title('汉宁窗函数曲线')
else
    plot(2*nf/Nf,Hk,2*nf/Nf,Hk,'*')
    axis([-0.1,2.1,-0.1,1.1])
    xlabel('w/\pi')
    ylabel('Hg(w)')
    title('理想低通特性曲线')
end
axes(handles.axes7)
plot(w/pi,20*log10(abs(Hw)))
axis([0,1,min(20*log10(abs(Hw))),10])
xlabel('w/\pi')
ylabel('20lg|H(w)|')
title('系统衰减函数')
set(handles.pushbutton5,'Enable','on');

%显示滤波后波形
function pushbutton5_Callback(hObject, eventdata, handles)
global t n y hn N Fs
z=fftfilt(hn,y);
sound (z,Fs)
H3=fft(z,N);
axes(handles.axes4)
plot(t,z)
xlabel('t')
ylabel('z(t)')
title('滤波后信号')
axes(handles.axes8)
plot(n*Fs/N,abs(H3))
xlabel('f')
ylabel('|Hg(\Omega)|')
title('滤波后信号频谱')

function edit8_Callback(hObject, eventdata, handles)
global wp Fs
fp=get(handles.edit8,'string');
fp=str2num(fp);
wp=2*pi*fp/Fs;


function edit9_Callback(hObject, eventdata, handles)
global ws Fs
fs=get(handles.edit9,'string');
fs=str2num(fs);
ws=2*pi*fs/Fs;


function edit10_Callback(hObject, eventdata, handles)
global wup Fs
fup=get(handles.edit10,'string');
fup=str2num(fup);
wup=2*pi*fup/Fs;


function edit11_Callback(hObject, eventdata, handles)
global wus Fs
fus=get(handles.edit10,'string');
fus=str2num(fus);
wus=2*pi*fus/Fs;

function gui1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to gui1 (see VARARGIN)
% Choose default command line output for gui1
handles.output = hObject;
% Update handles structure
guidata(hObject, handles);
% UIWAIT makes gui1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);
% --- Outputs from this function are returned to the command line.
function varargout = gui1_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Get default command line output from handles structure
varargout{1} = handles.output;
function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double
% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double
% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double
% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double
% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double
% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1
% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% --- Executes during object creation, after setting all properties.
function text2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to text2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% --- Executes during object deletion, before destroying properties.
function text2_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to text2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% --- Executes on selection change in popupmenu2.
function popupmenu2_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu2
% --- Executes during object creation, after setting all properties.
function popupmenu2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% --- Executes during object creation, after setting all properties.
function popupmenu3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to choosethenoice (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% --- Executes during object creation, after setting all properties.
function popupmenu5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% --- Executes during object creation, after setting all properties.
function popupmenu6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
function edit10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit12_Callback(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of edit12 as text
%        str2double(get(hObject,'String')) returns contents of edit12 as a double
% --- Executes during object creation, after setting all properties.
function edit12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit13_Callback(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of edit13 as text
%        str2double(get(hObject,'String')) returns contents of edit13 as a double
% --- Executes during object creation, after setting all properties.
function edit13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
