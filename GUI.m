function varargout = GUI(varargin)
% GUI MATLAB code for GUI.fig
%      GUI, by itself, creates a new GUI or raises the existing
%      singleton*.
%
%      H = GUI returns the handle to a new GUI or the handle to
%      the existing singleton*.
%
%      GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI.M with the given input arguments.
%
%      GUI('Property','Value',...) creates a new GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUI

% Last Modified by GUIDE v2.5 17-Mar-2013 19:10:15

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @GUI_OpeningFcn, ...
    'gui_OutputFcn',  @GUI_OutputFcn, ...
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
% End initialization code - DO NOT EDIT


% --- Executes just before GUI is made visible.
function GUI_OpeningFcn(hObject, ~, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GUI (see VARARGIN)

% Choose default command line output for GUI
handles.output = hObject;



handles.mytimer = timer('Period',0.02,'TimerFcn',{@update,hObject,handles},'ExecutionMode','fixedRate');%,'BusyMode','queue');

handles.simulation = Simulation(handles.axes1);

set(gcf,'CloseRequestFcn',@my_closefcn);
%handles.x=5;
set(handles.popupnames,'String',fieldnames(handles.simulation.TheAircraft(1).controller.variables));
% Update handles structure
guidata(hObject, handles);
% UIWAIT makes GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);




function my_closefcn(hObject,eventdata)
data=guidata(hObject);
if strcmp(data.mytimer.Running,'off')
    fprintf('Closing\n');
    delete(data.mytimer);
    delete(data.simulation);
    guidata(hObject,data);
    delete(hObject);
else
    fprintf('Please stop simulation first\n')
end



% --- Outputs from this function are returned to the command line.
function varargout = GUI_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in runbutton.
function runbutton_Callback(hObject, eventdata, handles)
% hObject    handle to runbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%if (handles.mytimer.
if (strcmp(handles.mytimer.Running,'off'))
    start(handles.mytimer);
else
    stop(handles.mytimer);
end
guidata(hObject, handles);

% --- Executes on button press in fastforwardbutton.
function fastforwardbutton_Callback(hObject, eventdata, handles)
% hObject    handle to fastforwardbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
stop(handles.mytimer);
handles.mytimer.Period = 0.001;
start(handles.mytimer);
guidata(hObject, handles);


% --- Executes on button press in realtimebutton.
function realtimebutton_Callback(hObject, eventdata, handles)
% hObject    handle to realtimebutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
stop(handles.mytimer);
handles.mytimer.Period = 0.02;
start(handles.mytimer);
guidata(hObject, handles);

function update(timerObj,event,hObject,handles)
handles=guidata(hObject);
handles.simulation.Update(0.02);
%fprintf('%f\n',timerObj.Period);
drawnow;
% Update handles structure
guidata(hObject, handles);


% --- Executes on selection change in listnames.
function listnames_Callback(hObject, eventdata, handles)
% hObject    handle to listnames (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listnames contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listnames


% --- Executes during object creation, after setting all properties.
function listnames_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listnames (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editvariables_Callback(hObject, eventdata, handles)
% hObject    handle to editvariables (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editvariables as text
%        str2double(get(hObject,'String')) returns contents of editvariables as a double


% --- Executes during object creation, after setting all properties.
function editvariables_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editvariables (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupnames.
function popupnames_Callback(hObject, eventdata, handles)
% hObject    handle to popupnames (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupnames contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupnames
x=1;
string=get(handles.popupnames,'String');
variable_name=string{get(handles.popupnames,'Value')};
set(handles.editvariables,'String',handles.simulation.TheAircraft(1).controller.variables.(variable_name));

% --- Executes during object creation, after setting all properties.
function popupnames_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupnames (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in updatebutton.
function updatebutton_Callback(hObject, eventdata, handles)
% hObject    handle to updatebutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%variblename =
string=get(handles.popupnames,'String');
variable_name=string{get(handles.popupnames,'Value')};

variable_value=str2double(get(handles.editvariables,'String'));
if ~isnan(variable_value)
    for i=1:length(handles.simulation.TheAircraft)
        handles.simulation.TheAircraft(i).controller.update_variable(variable_name,variable_value);
    end
end