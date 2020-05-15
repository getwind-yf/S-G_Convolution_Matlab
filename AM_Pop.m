

function varargout = AM_Pop(varargin)
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @AM_Pop_OpeningFcn, ...
                   'gui_OutputFcn',  @AM_Pop_OutputFcn, ...
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

function AM_Pop_OpeningFcn(hObject, eventdata, handles, varargin)
handles.output = hObject;
guidata(hObject,handles);

function varargout = AM_Pop_OutputFcn(hObject, eventdata, handles) 
varargout{1} = handles.output;

function pushbutton1_Callback(hObject, eventdata, handles)
%1. Input data 
[filename,pathname]=uigetfile('*.xlsx','choisir des donnees'); %文件打开框
if isequal(filename,0)
    msgbox('Choisir le fichier EXCEL,SVP','Erreur');
end
delta0 = xlsread(strcat(pathname,filename),'sheet1','B2:B65536')/1000;
force0 = xlsread(strcat(pathname,filename),'sheet1','A2:A65536');
[point_max,loc_max]=findpeaks(force0);  %rechercher les points maximum et ses numéros d'ordre
[point_min,loc_min]=findpeaks(-force0);  %rechercher les points minimum et ses numéros d'ordre
point_min=-point_min;
index=find(point_max>(max(force0)+min(force0))/2);   
point_max=point_max(index);  
loc_max=loc_max(index);
index=find(point_min<(max(force0)+min(force0))/2);   
point_min= point_min(index);
loc_min=loc_min(index);

for i=1:min(length(loc_max),length(loc_min))-1     %rechercher le premier point maximum et son numéro d'ordre
    if loc_max(i+1)-loc_max(i)>abs(loc_max(1)-loc_min(1))
    a1=loc_max(i);
    break
    end
end

for j=i:min(length(loc_max),length(loc_min))-1     %rechercher le premier point minimum et son numéro d'ordre
      if loc_min(j+1)-loc_min(j)>abs(loc_max(1)-loc_min(1))
      b1=loc_min(j);
      break
      end
end

num_seg=ceil(800/(abs(a1-b1)));     %nombre de segments(à cette condition: le total des points dans tous les segments doivent être plus de 800)
num_point=abs(a1-b1);  %nombre despoints dans chaque segment

a=[];b=[b1];    
for i=1:num_seg
    [force_max,delta_ab]=max(force0(b(i):b(i)+2*num_point));
    a=[a,b(i)+delta_ab];     
    [force_min,delta_ba]=min(force0(a(i):a(i)+2*num_point));
    b=[b,a(i)+delta_ba];
end

force_acquis=[];delta_acquis=[];   %obtenir la force et le delta (à cette condition: le total des points dans tous les segments doivent être plus de 800)
for i=1:num_seg
    force_acquis=[force_acquis;force0(b(i):a(i))];
    delta_acquis=[delta_acquis;delta0(b(i):a(i))];
end

[force,index]=sort(force_acquis);
delta=delta_acquis(index);

%2. calcul la valeur de alpha 1
% filtre delta 1 par méthode de Savitzky-Golay
alpha1= polyfit(force,delta,1);
delta1=delta-alpha1(1)*force;
DF=[delta1,force];
delta1_filtre=sgolayfilt(delta1,4,31);

%3. régression polynomial d'un ordre 4
a= 4;
P= polyfit(force,delta1_filtre,a); 
%coefficients(des bornes de confiance à 95%):

%4 calcul de minimum et maximum dans la borne de Force
dP= polyder(polyder(P));    %coefficient de la dérivée seconde
dP_Val=polyval(dP,force);   %dP est la valeur de delta1 
[diff_min,b]=min(abs(dP_Val(1:ceil(0.8*length(force)))));   %b est le numéro d'ordre du point d'inflexion


for j=1:(b-1);
    c=cov(delta1_filtre(b-j:b+j),force(b-j:b+j));
    if abs(c(2,1))>0.05/1000
        d=j;  %d est la différence de numéro d'ordre entre le point d'inflexion et la borne
        break;
    end;
end


Force_borne_min=force(b-j);
Force_borne_max=force(b+j);

set(handles.text_Fborne_min,'string',num2str(Force_borne_min,4));
set(handles.text_Fborne_max,'string',num2str(Force_borne_max,4));
set(handles.slider_x1,'value',Force_borne_min);
set(handles.slider_x2,'value',Force_borne_max);
set(handles.text_x1,'string',num2str(Force_borne_min,4));
set(handles.text_x2,'string',num2str(Force_borne_max,4));

axes(handles.axes1)  %Affichage de la courbe Force-Delta
hold off
scatter(delta1,force,21,'.','g')
hold on 
plot(delta1_filtre,force);
xlim=get(gca,'xlim');
ylim=get(gca,'ylim');
hold on

plot(delta1_filtre(b),force(b),'*','color','r')
hold on

xx=0:0.1:max(ylim);
yy=polyval(P,xx);
line(yy,xx,'color','r') 
set(gca,'xlim',xlim,'ylim',ylim)
hold on


plot(xlim,[Force_borne_min Force_borne_min],'m:')
plot(xlim,[Force_borne_max Force_borne_max],'m:')
set(gca,'xlim',xlim,'ylim',ylim)

handles.b=b;   
handles.d=d;
handles.delta1_filtre=delta1_filtre;
handles.delta1=delta1;
handles.delta=delta;
handles.force=force;
handles.xx=xx;
handles.yy=yy;
handles.Force_borne_min=Force_borne_min
handles.Force_borne_max=Force_borne_max
guidata(hObject,handles)

function pushbutton_tr_Callback(hObject, eventdata, handles)
b=handles.b;
d=handles.d;
delta1_filtre=handles.delta1_filtre;
delta=handles.delta;
delta1=handles.delta1;
force=handles.force;
x1=handles.x1;
x2=handles.x2;

if x1==-1||x2==-1 
    d1=d;
    d2=d;
else
    
for i=1:length(force)
    if force(i)-x1>0
       b1=i;  
     break;
   end;
end

for i=1:length(force)
    if force(i)-x2>0
      b2=i-1; 
     break;
    end;
end
d1=b-b1;
d2=b2-b;
end


%6. calcul de alpha2
alpha2=polyfit(force(b-d1:b+d2),delta1_filtre(b-d1:b+d2),1);
delta2_filtre = delta1_filtre - alpha2(1)*force;

%7. moyenne de delta
delta2_moy=mean(delta2_filtre(b-d1:b+d2));

%8. calcul de la valeur de a et de b
Y = (delta2_filtre(1:b-d1)-min(delta2_filtre))/(delta2_moy-min(delta2_filtre));
index = find(Y>=Y(1)&Y<1);
X = log(1-Y(index));
a = polyfit(X,force(index),1);
F = a(1)*X+a(2);

axes(handles.axes2)
hold off
axes(handles.axes2)
delta_droit=delta1- alpha2(1)*force
scatter(delta_droit,force,21,'.','g')
hold on

plot(delta2_filtre,force,'b',delta2_filtre(index),F,'r')
xlim=get(gca,'xlim')
ylim=get(gca,'ylim')
hold on 

%9. calcul de la valeur Fop
Fop=a(1)*log(0.02)+a(2)
Fop_x=(1-0.02)*(delta2_moy-min(delta2_filtre))+min(delta2_filtre)
set(handles.text_Fop,'string',num2str(Fop,3))



plot(Fop_x,Fop,'o','color','r', 'MarkerSize',5)
hold on 
set(gca,'xlim',xlim,'ylim',ylim)
plot([Fop_x Fop_x],ylim,'black:')
text(delta2_moy-(xlim(2)-xlim(1))*0.3,ylim(1)+0.3,'2% offset \rightarrow ','FontSize',10)
plot([delta2_moy delta2_moy],ylim,'black:')
text(delta2_moy,ylim(1)+0.3,' \leftarrow delta-moy','FontSize',10)
set(handles.text_moy,'string',num2str(delta2_moy,6))
plot([xlim(1) Fop_x],[Fop Fop],'r:')
hold on

guidata(hObject,handles)

function axes1_CreateFcn(hObject, eventdata, handles)

function slider_x1_Callback(hObject, eventdata, handles)
b=handles.b;   
d=handles.d;
delta1_filtre=handles.delta1_filtre;
delta1=handles.delta1;
delta=handles.delta;
force=handles.force;
xx=handles.xx;
yy=handles.yy;
x2=handles.x2
Force_borne_min=handles.Force_borne_min
Force_borne_max=handles.Force_borne_max

x1=get(handles.slider_x1,'value');
handles.x1=x1;
set(handles.text_x1,'string',num2str(x1,4));

axes(handles.axes1)
hold off
scatter(delta1,force,21,'.','g')
hold on

Force_delta=plot(delta1_filtre,force)
xlim=get(gca,'xlim');
ylim=get(gca,'ylim');
hold on

axes(handles.axes1)
plot(delta1_filtre(b),force(b),'*','color','r') 
hold on

axes(handles.axes1)
line(yy,xx,'color','r')
set(gca,'xlim',xlim,'ylim',ylim)
hold on

plot(xlim,[x1 x1],'r:');
plot(xlim,[x2 x2],'b:');
guidata(hObject,handles);

function slider_x1_CreateFcn(hObject, eventdata, handles)


x1=-1;
handles.x1=x1;
guidata(hObject,handles);
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function slider_x2_Callback(hObject, eventdata, handles)
x2=get(handles.slider_x2,'value');
handles.x2=x2;
set(handles.text_x2,'string',num2str(x2,4));

b=handles.b;   
d=handles.d;
delta1_filtre=handles.delta1_filtre;
delta1=handles.delta1;
delta=handles.delta;
force=handles.force;
xx=handles.xx;
yy=handles.yy;

x1=handles.x1;

axes(handles.axes1)
hold off
scatter(delta1,force,21,'.','g')
hold on
Force_delta=plot(delta1_filtre,force)
xlim=get(gca,'xlim');
ylim=get(gca,'ylim');
hold on

axes(handles.axes1)
plot(delta1_filtre(b),force(b),'*','color','r') 
hold on

axes(handles.axes1)
line(yy,xx,'color','r')
set(gca,'xlim',xlim,'ylim',ylim)
hold on

ylim=get(gca,'ylim');
plot(xlim,[x1 x1],'r:');
plot(xlim,[x2 x2],'b:')


guidata(hObject,handles);

function slider_x2_CreateFcn(hObject, eventdata, handles)
x2=-1;
handles.x2=x2;
guidata(hObject,handles);
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function text_x2_CreateFcn(hObject, eventdata, handles)

function text_x1_CreateFcn(hObject, eventdata, handles)

function axes2_CreateFcn(hObject, eventdata, handles)

function text_Fborne_min_CreateFcn(hObject, eventdata, handles)

function text_Fborne_max_CreateFcn(hObject, eventdata, handles)

function text14_CreateFcn(hObject, eventdata, handles)

function text_Fop_CreateFcn(hObject, eventdata, handles)

function text_moy_CreateFcn(hObject, eventdata, handles)


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over slider_x1.


% --- Executes during object deletion, before destroying properties.
