function varargout = plots_occupancy(varargin)
% PLOTS MATLAB code for plots.fig
%      PLOTS, by itself, creates a new PLOTS or raises the existing
%      singleton*.
%
%      H = PLOTS returns the handle to a new PLOTS or the handle to
%      the existing singleton*.
%
%      PLOTS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PLOTS.M with the given input arguments.
%
%      PLOTS('Property','Value',...) creates a new PLOTS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before plots_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to plots_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help plots

% Last Modified by GUIDE v2.5 11-Oct-2018 09:24:08

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @plots_OpeningFcn, ...
                   'gui_OutputFcn',  @plots_OutputFcn, ...
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

% --- Executes just before plots is made visible.
function plots_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to plots (see VARARGIN)

% Choose default command line output for plots
handles.output = hObject;
cameratoolbar(hObject);
handles.hScatter = gobjects(1,100);
handles.hClust   = gobjects(1,100);

mode = 1;
handles.mode = mode;
handles.showDetObj = 1;
handles.showClusters = 1;

% Update defaults depending on mode
    set(handles.uitable2, 'Data', [0 3 0.5; 0 180 30]);
    set(handles.uitable1, 'Data', [-3 3; 0 3;-1 1]);
    set(handles.popupmenu_Main_Plot, 'String', {'3D', 'Top Down/Depth vs Width (2D - YX)', 'Elevation vs Depth (2D - ZY)', 'Elevation vs Width (2D - ZX)'});


% Initialize main plot
axes(handles.axes1);
handles = initMainPlot(hObject, handles, mode);

% Update handles structure
guidata(hObject, handles);

% This sets up the initial plot - only do when we are invisible
% so window can get raised using plots.
if strcmp(get(hObject,'Visible'),'off')
    plot(rand(5));
end

% UIWAIT makes plots wait for user response (see UIRESUME)
uiwait(hObject);


% --- Outputs from this function are returned to the command line.
function varargout = plots_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles;
varargout{2} = handles.hScatter;
varargout{3} = handles.axes1;
varargout{4} = handles.axes1.UserData.hPatch;
varargout{5} = handles.textNumObj;
varargout{6} = handles.hClust;

% --- Executes on button press: "Apply All Plot Settings".
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%axes(handles.axes1);
%cla;
axisLimits = reshape(handles.uitable1.Data', 1,[]);
axis(handles.axes1, axisLimits)
%daspect([1 1 1])
% xlabel('X'), ylabel('Y'), zlabel('Z')
% disp(view())
handles = getVisualizerSettings(handles);
handles = updateMainPlot(hObject, handles, handles.mode);

guidata(hObject, handles)


% --------------------------------------------------------------------
function FileMenu_Callback(hObject, eventdata, handles)
% hObject    handle to FileMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function OpenMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to OpenMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
file = uigetfile('*.fig');
if ~isequal(file, 0)
    open(file);
end

% --------------------------------------------------------------------
function PrintMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to PrintMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
printdlg(handles.figureViz)

% --------------------------------------------------------------------
function CloseMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to CloseMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selection = questdlg(['Close ' get(handles.figureViz,'Name') '?'],...
                     ['Close ' get(handles.figureViz,'Name') '...'],...
                     'Yes','No','Yes');
if strcmp(selection,'No')
    return;
end

delete(handles.figureViz)


% --- Executes on selection change in popupmenu_Main_Plot.
function popupmenu_Main_Plot_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_Main_Plot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns popupmenu_Main_Plot contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_Main_Plot


% --- Executes during object creation, after setting all properties.
function popupmenu_Main_Plot_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_Main_Plot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
     set(hObject,'BackgroundColor','white');
end
set(hObject, 'String', {'3D', '2D - XY', '2D - YZ', '2D - XZ'});


% --- Executes during object creation, after setting all properties.
function uitable1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to uitable1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
set(hObject, 'Data', [-5 5; 0 5; -2 0.5]);
set(hObject, 'RowName', {'X', 'Y', 'Z'}, 'ColumnName', {'Min', 'Max'});

function hPatch = cube_plot(ax, origin,X,Y,Z,color)

ver = [1 1 0;
    0 1 0;
    0 1 1;
    1 1 1;
    0 0 1;
    1 0 1;
    1 0 0;
    0 0 0];

%  Define the faces of the unit cubic
fac = [1 2 3 4;
    4 3 5 6;
    6 7 8 5;
    1 2 8 7;
    6 7 1 4;
    2 3 5 8];

cube = [ver(:,1)*X+origin(1),ver(:,2)*Y+origin(2),ver(:,3)*Z+origin(3)];
hPatch = patch(ax, 'Faces',fac,'Vertices',cube,'FaceColor',color);


% --- Executes during object creation, after setting all properties.
function axes2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes2


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in checkboxEnableOccupancy.
function checkboxEnableOccupancy_Callback(hObject, eventdata, handles)
% hObject    handle to checkboxEnableOccupancy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkboxEnableOccupancy


% --- Executes on selection change in popupmenuPersistance.
function popupmenuPersistance_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenuPersistance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenuPersistance contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenuPersistance
a = [0 1 5 10 1000];
handles.maxHist = a(get(hObject,'Value'));

% --- Executes during object creation, after setting all properties.
function popupmenuPersistance_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenuPersistance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu6.
function popupmenu6_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu6 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu6


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


% --- Executes on selection change in popupmenuGridFill.
function popupmenuGridFill_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenuGridFill (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenuGridFill contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenuGridFill


% --- Executes during object creation, after setting all properties.
function popupmenuGridFill_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenuGridFill (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function [handles] = initMainPlot(hObject, handles, mode)
axes(handles.axes1);
% color = colormap(handles.axes1,parula(10));
% handles.axes0CLim = handles.uitable1.Data(3,:);
% colorbar
%cla;
%common to either mode
%create scatter plot/object
for i=1:size(handles.hScatter,2)
    handles.hScatter(i) = scatter3(handles.axes1, 0,0,0, 25, 'filled');%'MarkerFaceColor', color(i,:),'MarkerEdgeColor','none');
    hold(handles.axes1, 'on')
end
%create cluster plot/object
%  Define the faces of the unit cubic
fac = [1 2 3 4;
       4 3 5 6;
       6 7 8 5;
       1 2 8 7;
       6 7 1 4;
       2 3 5 8];

cube = zeros(8, 3);

for i=1:size(handles.hClust,2)
    handles.hClust(i) = patch(handles.axes1, 'Faces',fac,'Vertices',cube,'EdgeColor', [0 0 0],'FaceAlpha',0,'FaceColor','none','LineWidth',2);
    hold(handles.axes1, 'on')
end

hold(handles.axes1, 'off')
color = colormap(handles.axes1,parula(10));
handles.axes1.CLim = handles.uitable1.Data(3,:);
c = colorbar(handles.axes1);
handles.hColorbar = c;
handles.hColorbar.Label.String = 'Z (m)';
s = struct('hPatch',gobjects(1,9), 'typePatchFill', 1, 'maxHist', 1, 'typeMarkerSize', 1, 'typeMarkerColor', 1, 'gridLim', handles.uitable2.Data, 'gridHist', zeros([3, 3, 3]), 'mode',2, 'showDetObj',1, 'showClusters',1);
set(handles.axes1, 'UserData', s);

%axis settings
axisLimits = reshape(handles.uitable1.Data', 1,[]);
axis(handles.axes1, axisLimits)
axis(handles.axes1, 'manual')
set(handles.axes1, 'color', 'k')
xlabel('X'), ylabel('Y'), zlabel('Z')
handles.axes1.GridColor = [1, 1, 1];
handles.axes1.LineWidth = 2.0;
handles.axes1.XColor = [1 0 0];
handles.axes1.YColor = [0 0 1];
handles.axes1.ZColor = [0 .7 0];
handles.axes1.FontUnits = 'normalized';
handles.axes1.FontSize = 0.03;
%set 3D view
daspect(handles.axes1, [1 1 1])
%camproj('orthographic')
if(mode == 1)%side
    view(handles.axes1, [-140, 15])
elseif (mode == 2) %overhead
    view(handles.axes1, [40, 10])
    handles.axes1.CameraUpVector = [0 -1 0];
end

%draw EVM
hEVMPower = cube_plot(handles.axes1, [-0.15 0.0 -0.1], 0.05,0.1,0.1, 'b');
hEVM = cube_plot(handles.axes1, [-.1 0 -.15], 0.2,0.05,0.3, [0.8 0 0]);
hEVMAntenna = cube_plot(handles.axes1, [-.05 .05 0.05], 0.1,0.025,0.1, 'y');


%drawOccupancyGrid
if(mode == 1)
    drawPlane = axisLimits(5);
elseif(mode == 2)
    drawPlane = axisLimits(4);
end
hPatch = drawOccupancyGrid(handles.axes1, handles.uitable2.Data(1,:), handles.uitable2.Data(2,:), mode, drawPlane);
handles.axes1.UserData.hPatch = hPatch;
handles.axes1.UserData.gridLim = handles.uitable2.Data;
handles.axes1.UserData.gridHist = zeros([size(hPatch) 1]);
handles.axes1.UserData.gridElev = zeros([size(hPatch) 1]);
guidata(hObject, handles)


function [hPatch] = drawOccupancyGrid(ax, range, angle, mode, drawPlane)
% drawPlane = the z or y value at which the grid is drawn. For side mount
% drawPlane should be minZ, overhead mount is maxY
angleStep = angle(3); %degrees
angleLim = angle(1:2);
rangeStep = range(3);
rangeLim = range(1:2);
arcStep = floor(angleStep/10*5);
hPatch = {};
for r=rangeLim(1):rangeStep:rangeLim(2)-rangeStep
    for t=angleLim(1):angleStep:angleLim(2)-angleStep
        theta = deg2rad(t:angleStep/arcStep:t+angleStep);
        rho1 = repelem(r,numel(theta));
        rho2 = repelem(r+rangeStep,numel(theta));
        [x1 y1] = pol2cart(theta,rho1);
        [x2 y2] = pol2cart(theta,rho2);
        if (mode == 1) %side mount
            p = patch(ax, 'XData',[x1 fliplr(x2)], 'YData', [y1 fliplr(y2)], 'ZData', repelem(drawPlane, numel(x1)*2));
        elseif (mode == 2) %overhead mount
            p = patch(ax, 'XData',[x1 fliplr(x2)], 'ZData', [y1 fliplr(y2)], 'YData', repelem(drawPlane, numel(x1)*2));
        end
        p.FaceColor = 'none';
        p.EdgeColor = 'w';
        hPatch{end+1} = p;
    end
end
%reshape so index of hPatch correlates to grid coordinates
hPatch = reshape(hPatch, [], (rangeLim(2)-rangeLim(1))/rangeStep);



% This function is called whenever a new plot type (3D, 2D, etc.) is selected
function [handles] = updateMainPlot(hObject, handles, mode)

axes(handles.axes1);

cube = zeros(8, 3);

% reset clusters to origin
for i=1:size(handles.hClust,2)
    set(handles.hClust(i), 'Vertices', cube, 'EdgeColor', [0 0 0]);
end

%axis settings
axisLimits = handles.uitable1.Data;
handles.axes1.XLim = axisLimits(1,:);
handles.axes1.YLim = axisLimits(2,:);
handles.axes1.ZLim = axisLimits(3,:);
%handles.axes1.CLim = axisLimits(3,:);

%set 3D view
daspect(handles.axes1, [ 1 1 1])

if(mode == 1)%side
    view(handles.axes1, [-140, 15])
elseif (mode == 2) %overhead

end

popup_sel_index = get(handles.popupmenu_Main_Plot, 'Value');
if(mode==1) %overhead
    switch popup_sel_index
        case 1 %3D
             view(handles.axes1, [-140, 15])
             %handles.axes1.CameraUpVector = [0 0 1];
        case 2 %top down/YX
            view(handles.axes1, [0 90])
            %handles.axes1.CameraUpVector = [0 0 1];
        case 3 %ZY
            view(handles.axes1, [90, 0])
            %handles.axes1.CameraUpVector = [0 0 1];
        case 4 %ZX
            view(handles.axes1, [180, 0])
            %handles.axes1.CameraUpVector = [0 0 1];
    end
elseif(mode==2)
    switch popup_sel_index
        case 1 %3D
            view(handles.axes1, [40, 10])
            handles.axes1.CameraUpVector = [0 -1 0];
        case 2 %top down ZX
            view(handles.axes1, [0, 0])
            handles.axes1.CameraUpVector = [0 0 1];
        case 3 %YZ
            view(handles.axes1, [90, 0])
            handles.axes1.CameraUpVector = [0 -1 0];
        case 4 %YX
            view(handles.axes1, [0, -90])
            handles.axes1.CameraUpVector = [0 -1 0];
    end
end

psize = size(handles.axes1.UserData.hPatch);

for idx1=1:psize(1)
  for idx2=1:psize(2)
    delete(handles.axes1.UserData.hPatch{idx1, idx2});
  end
end

if(mode == 1)
    drawPlane = axisLimits(3,1);
elseif(mode == 2)
    drawPlane = axisLimits(2,2);
end
hPatch = drawOccupancyGrid(handles.axes1, handles.uitable2.Data(1,:), handles.uitable2.Data(2,:), mode, drawPlane);
handles.axes1.UserData.hPatch = hPatch;
handles.axes1.UserData.gridLim = handles.uitable2.Data;
handles.axes1.UserData.gridHist = zeros([size(hPatch) handles.axes1.UserData.maxHist]);
handles.axes1.UserData.gridElev = zeros([size(hPatch) handles.axes1.UserData.maxHist]);
%drawEVM
hEVMPower = cube_plot(handles.axes1, [-0.15 0.0 -0.1], 0.05,0.1,0.1, 'b');
hEVM = cube_plot(handles.axes1, [-.1 0 -.15], 0.2,0.05,0.3, [0.8 0 0]);
hEVMAntenna = cube_plot(handles.axes1, [-.05 .05 0.05], 0.1,0.025,0.1, 'y');

guidata(hObject, handles)


% --- Executes during object creation, after setting all properties.
function uitable2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to uitable2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
set(hObject, 'Data', [0 5 1; 0 360 20]);
set(hObject, 'RowName', {'Range', 'Angle'}, 'ColumnName', {'Min', 'Max','Step'});

function [handles] = getVisualizerSettings(handles)
popup_sel_index = get(handles.popupmenu_Main_Plot, 'Value');
mode = handles.mode;

handles.showDetObj = get(handles.radiobuttonDetObj,  'Value');
handles.axes1.UserData.showDetObj = handles.showDetObj;
handles.showClusters = get(handles.radiobuttonClusters,  'Value');
handles.axes1.UserData.showClusters = handles.showClusters;

% marker size
typeMarkerSize = 25;
handles.axes1.UserData.typeMarkerSize = typeMarkerSize;

% marker color
typeMarkerColor = 1;
handles.axes1.UserData.typeMarkerColor = typeMarkerColor;
switch typeMarkerColor
    case 1
        %Z
        color = colormap(handles.axes1,parula(10));
        handles.axes1.CLim = handles.axes1.ZLim;
        set(handles.hColorbar,'Visible','on','TicksMode', 'auto', 'TickLabelsMode','auto','Location', 'eastoutside')
        handles.hColorbar.Label.String = 'Z (m)';

    case 2
        %Y
        color = colormap(handles.axes1,parula(10));
        handles.axes1.CLim = handles.axes1.YLim;
        set(handles.hColorbar, 'Visible','on', 'TicksMode', 'auto', 'TickLabelsMode','auto', 'Location', 'eastoutside')
        handles.hColorbar.Label.String = 'Y (m)';
    case 3
        %X
        color = colormap(handles.axes1,parula(10));
        handles.axes1.CLim = handles.axes1.XLim;
        set(handles.hColorbar, 'Visible','on','TicksMode', 'auto', 'TickLabelsMode','auto', 'Location', 'eastoutside')
        handles.hColorbar.Label.String = 'X (m)';
    case 4
        color = colormap(handles.axes1,parula(3));
        handles.axes1.CLim = [1 3];
        set(handles.hColorbar, 'Visible','on','Location', 'eastoutside', 'Ticks',[1,2,3], 'TickLabels', {'-', '0', '+'})
        handles.hColorbar.Label.FontSize = 20;
        handles.hColorbar.Label.FontWeight = 'bold';
        handles.hColorbar.Label.String = 'Doppler Sign';
    case 5
        %Time
        if(maxHist > 1)
            map = [repmat([0.2 0.0], maxHist, 1) linspace(1,0.2,maxHist)'];
            color = colormap(handles.axes1,map);
            handles.axes1.CLim = [1 maxHist] ;
            set(handles.hColorbar,'Visible','on', 'Ticks', [1 maxHist], 'TickLabelsMode','auto', 'Location', 'eastoutside')
            handles.hColorbar.Label.String = 'Time/Frame #';
        else
            set(handles.hColorbar, 'Visible','off')
        end

    case 6
        %Constant
        set(handles.hColorbar, 'Visible','off')
end



% --- Executes during object creation, after setting all properties.
function figureViz_CreateFcn(hObject, eventdata, handles)
% hObject    handle to figureViz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called



% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes1


% --- Executes on button press in pushbuttonBrowse.
function pushbuttonBrowse_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonBrowse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[filename, pathname] = uigetfile('*.cfg','*.*');
configurationFileName = [pathname filename];
handles.cfg.filename = configurationFileName;

if (filename ~= 0)
    % Read Chirp Configuration file
    cliCfg = readCfg(handles.cfg.filename);
    [Params cliCfg] = parseCfg(cliCfg);

    % Display chirp table
    hTable = findobj('Tag', 'uitableChirp');
    hTable = displayChirpParams(Params, hTable);

    % Update handles
    handles.cfg.cliCfg = cliCfg;
    handles.params = Params;
    guidata(hObject,handles)
end


% --- Executes on button press in pushbuttonLoad.
% --- For Park Assist, this is the "Start" button.
function pushbuttonLoad_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonLoad (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.mode = 1;
handles.axes1.UserData.mode = 1;
guidata(hObject, handles);

 if (length(instrfind('Type','serial', 'Status','open'))>=2 && ~isempty(handles.UARTCOM.hControlSerialPort) && ~isempty(handles.DATACOM.hDataSerialPort))

    %load Cfg Params
    mmwDemoCliPrompt = char('MrrTIDesign:/>');

[Params ] = makeCfg();
% Update handles
%handles.cfg.cliCfg = cliCfg;
handles.params = Params;
guidata(hObject,handles)

    %Send CLI configuration to IWR16xx
    fprintf('Starting configuration in AWR18xx ...\n');

    command(1) = "advFrameCfg";
    command(2) = "sensorStart";

    for k=1:2
        fprintf(handles.UARTCOM.hControlSerialPort, command(k));
        fprintf('%s\n', command(k));
        echo = fgetl(handles.UARTCOM.hControlSerialPort); % Get an echo of a command
        done = fgetl(handles.UARTCOM.hControlSerialPort); % Get "Done"
        prompt = fread(handles.UARTCOM.hControlSerialPort, size(mmwDemoCliPrompt,2)); % Get the prompt back
    end
    %fclose(handles.UARTCOM.hControlSerialPort);
    %delete(hControlSerialPort);

    plots_OutputFcn(hObject,eventdata,guidata(hObject));
    uiresume(handles.figureViz);
else
    warndlg('Error: Cannot start COM ports not connected. Please select and connect.');
end


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
hEditUART = findobj('Tag','editUART');
hEditData = findobj('Tag','editData');

handles.UARTCOM.num = str2num(hEditUART.String);
handles.DATACOM.num = str2num(hEditData.String);

% Clear ports
if ~isempty(instrfind('Type','serial'))
    disp('Serial port(s) already open. Re-initializing...');
    delete(instrfind('Type','serial'));  % delete open serial ports.
end

% Configure data port
hDataSerialPort = configureDataPort(handles.DATACOM.num, 65536);
% Configure UART port
hControlSerialPort = configureControlPort(handles.UARTCOM.num);

% Update COM status msg
hCOMStatus = findobj('Tag', 'textCOMStatus');
update = 'COM STATUS: Ports connected';
set(hCOMStatus,'String', update);

%Update Handles
handles.DATACOM.hDataSerialPort = hDataSerialPort;
handles.UARTCOM.hControlSerialPort = hControlSerialPort;
guidata(hObject,handles);


function editData_Callback(hObject, eventdata, handles)
% hObject    handle to editData (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editData as text
%        str2double(get(hObject,'String')) returns contents of editData as a double


% --- Executes during object creation, after setting all properties.
function editData_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editData (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editUART_Callback(hObject, eventdata, handles)
% hObject    handle to editUART (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editUART as text
%        str2double(get(hObject,'String')) returns contents of editUART as a double


% --- Executes during object creation, after setting all properties.
function editUART_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editUART (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function [sphandle] = configureDataPort(comPortNum, bufferSize)

    comPortString = ['COM' num2str(comPortNum)];
    sphandle = serial(comPortString,'BaudRate',921600);
    set(sphandle,'Terminator', '');
    set(sphandle,'InputBufferSize', bufferSize);
    set(sphandle,'Timeout',10);
    set(sphandle,'ErrorFcn',@dispError);
    set(sphandle,'BytesAvailableFcnMode','byte');
    set(sphandle,'BytesAvailableFcnCount', 2^16+1);%BYTES_AVAILABLE_FCN_CNT);
    set(sphandle,'BytesAvailableFcn',@readUartCallbackFcn);
    fopen(sphandle);

function [sphandle] = configureControlPort(comPortNum)

    comPortString = ['COM' num2str(comPortNum)];
    sphandle = serial(comPortString,'BaudRate',115200);
    set(sphandle,'Parity','none')
    set(sphandle,'Terminator','LF')
    fopen(sphandle);


%Create relevant CLI parameters and store into P structure
%function [P cliCfg] = makeCfg()
function [P ] = makeCfg()
global TOTAL_PAYLOAD_SIZE_BYTES
global MAX_NUM_OBJECTS
global OBJ_STRUCT_SIZE_BYTES
global platformType
global STATS_SIZE_BYTES

    P=[];
    % 'channelCfg'
    P.channelCfg.txChannelEn = 7;
    P.dataPath.numTxAzimAnt = 2;
    P.dataPath.numTxElevAnt = 1;
    P.channelCfg.rxChannelEn = 15;
    P.dataPath.numRxAnt = 4;
    P.dataPath.numTxAnt = P.dataPath.numTxElevAnt + P.dataPath.numTxAzimAnt;

    % 'profileCfg'
    P.profileCfg.startFreq = 77;
    P.profileCfg.idleTime =  7;
    P.profileCfg.rampEndTime = 87.28;
    P.profileCfg.freqSlopeConst = 42.0;
    P.profileCfg.numAdcSamples = 512;
    P.profileCfg.digOutSampleRate = 6222;

    % 'frameCfg'
    P.frameCfg.chirpStartIdx = 128;
    P.frameCfg.chirpEndIdx = 130;
    P.frameCfg.numLoops = 32;
    P.frameCfg.numFrames = 0;
    P.frameCfg.framePeriodicity = 6000000;

    % 'guiMonitor'
    P.guiMonitor.detectedObjects = 1;
    P.guiMonitor.clusters = 1;
    P.guiMonitor.rangeAzimuthHeatMap = 0;
    P.guiMonitor.rangeElevHeatMap = 0;

    % 'dbscanCfg'
    P.dbScan.nAccFrames = 4;
    P.dbScan.epsilon = 4;
    P.dbScan.weight = 13;
    P.dbScan.vFactor = 20;
    P.dbScan.minPointsInCluster = 3;
    P.dbScan.fixedPointScale = 256;

    % 'cfarCfg'
    P.cfarCfg.detectMethod = 1;
    P.cfarCfg.leftSkipBin = 4;
    P.cfarCfg.closeInRangeBin = 12;
    P.cfarCfg.searchWinSizeRange = 4;
    P.cfarCfg.guardSizeRange = 2;
    P.cfarCfg.searchWinSizeSpreading = 8;
    P.cfarCfg.guardSizeSpreading = 2;
    P.cfarCfg.rangeThresh = 350;
    P.cfarCfg.fftSpreadingThresh = 30;
    P.cfarCfg.noiseCalcType = 2;
    P.cfarCfg.localPeakEnable = 0;
    P.cfarCfg.peakAngleDiffThresh = 5;
    P.cfarCfg.maxRangeForDetection = 20 * 0.1;

    P.dataPath.numChirpsPerFrame = (P.frameCfg.chirpEndIdx -...
                                    P.frameCfg.chirpStartIdx + 1) *...
                                    P.frameCfg.numLoops;
    P.dataPath.numDopplerBins = P.dataPath.numChirpsPerFrame / P.dataPath.numTxAnt;

    P.dataPath.rangeResolutionMeters = 3e8 * P.profileCfg.digOutSampleRate * 1e3 /...
                     (2 * P.profileCfg.freqSlopeConst * 1e12 * P.profileCfg.numAdcSamples);
    P.dataPath.numRangeBins = round(P.cfarCfg.maxRangeForDetection / P.dataPath.rangeResolutionMeters, 0);
P.dataPath.numRangeBins = 512;
    P.dataPath.rangeBinsPerMeter = P.dataPath.numRangeBins / P.cfarCfg.maxRangeForDetection;

    P.dataPath.rangeIdxToMeters = 3e8 * P.profileCfg.digOutSampleRate * 1e3 /...
                     (2 * P.profileCfg.freqSlopeConst * 1e12 * P.dataPath.numRangeBins);
    P.dataPath.dopplerResolutionMps = 3e8 / (2*P.profileCfg.startFreq*1e9 *...
                                        (P.profileCfg.idleTime + P.profileCfg.rampEndTime) *...
                                        1e-6 * P.dataPath.numDopplerBins * P.dataPath.numTxAnt);
    P.dataPath.maxRange = 300 * 0.9 * P.profileCfg.digOutSampleRate /(2 * P.profileCfg.freqSlopeConst * 1e3);
    P.dataPath.maxVelocity = 3e8 / (4*P.profileCfg.startFreq*1e9 *(P.profileCfg.idleTime + P.profileCfg.rampEndTime) * 1e-6 * P.dataPath.numTxAnt);

    %Calculate monitoring packet size
    tlSize = 8 %TL size 8 bytes
    TOTAL_PAYLOAD_SIZE_BYTES = 40; % size of header
    P.guiMonitor.numFigures = 1; %One figure for numerical parameers
    if P.guiMonitor.detectedObjects == 1 && P.guiMonitor.rangeAzimuthHeatMap == 1
        TOTAL_PAYLOAD_SIZE_BYTES = TOTAL_PAYLOAD_SIZE_BYTES +...
            OBJ_STRUCT_SIZE_BYTES*MAX_NUM_OBJECTS + tlSize;
        P.guiMonitor.numFigures = P.guiMonitor.numFigures + 1; %1 plots: X/Y plot
    end
    if P.guiMonitor.detectedObjects == 1 && P.guiMonitor.rangeElevHeatMap ~= 1
        TOTAL_PAYLOAD_SIZE_BYTES = TOTAL_PAYLOAD_SIZE_BYTES +...
            OBJ_STRUCT_SIZE_BYTES*MAX_NUM_OBJECTS + tlSize;
        P.guiMonitor.numFigures = P.guiMonitor.numFigures + 2; %2 plots: X/Y plot and Y/Doppler plot
    end
    if P.guiMonitor.clusters == 1
        TOTAL_PAYLOAD_SIZE_BYTES = TOTAL_PAYLOAD_SIZE_BYTES +...
            P.dataPath.numRangeBins * 2 + tlSize;
        P.guiMonitor.numFigures = P.guiMonitor.numFigures + 1;
    end
    if P.guiMonitor.rangeAzimuthHeatMap == 1
        TOTAL_PAYLOAD_SIZE_BYTES = TOTAL_PAYLOAD_SIZE_BYTES +...
            (P.dataPath.numTxAzimAnt * P.dataPath.numRxAnt) * P.dataPath.numRangeBins * 4 + tlSize;
        P.guiMonitor.numFigures = P.guiMonitor.numFigures + 1;
    end
    if P.guiMonitor.rangeElevHeatMap == 1
        TOTAL_PAYLOAD_SIZE_BYTES = TOTAL_PAYLOAD_SIZE_BYTES +...
            P.dataPath.numDopplerBins * P.dataPath.numRangeBins * 2 + tlSize;
        P.guiMonitor.numFigures = P.guiMonitor.numFigures + 1;
    end

    TOTAL_PAYLOAD_SIZE_BYTES = 32 * floor((TOTAL_PAYLOAD_SIZE_BYTES+31)/32);
    P.guiMonitor.numFigRow = 2;
    P.guiMonitor.numFigCol = ceil(P.guiMonitor.numFigures/P.guiMonitor.numFigRow);
    [P.dspFftScaleComp2D_lin, P.dspFftScaleComp2D_log] = dspFftScalCompDoppler(16, P.dataPath.numDopplerBins)
    [P.dspFftScaleComp1D_lin, P.dspFftScaleComp1D_log]  = dspFftScalCompRange(64, P.dataPath.numRangeBins)

    P.dspFftScaleCompAll_lin = P.dspFftScaleComp2D_lin * P.dspFftScaleComp1D_lin;
    P.dspFftScaleCompAll_log = P.dspFftScaleComp2D_log + P.dspFftScaleComp1D_log;

return


%Read relevant CLI parameters and store into P structure
function [P cliCfg] = parseCfg(cliCfg)
global TOTAL_PAYLOAD_SIZE_BYTES
global MAX_NUM_OBJECTS
global OBJ_STRUCT_SIZE_BYTES
global platformType
global STATS_SIZE_BYTES

    P=[];
    for k=1:length(cliCfg)
        C = strsplit(cliCfg{k});
        if strcmp(C{1},'channelCfg')
            P.channelCfg.txChannelEn = str2num(C{3});
            if platformType == hex2dec('a1642')
                % Hardcode for FSS EVM:
                P.dataPath.numTxAzimAnt = 1;
                P.dataPath.numTxElevAnt = 1;
            elseif (platformType == hex2dec('a1443') || platformType == hex2dec('a1111'))
                P.dataPath.numTxAzimAnt = bitand(bitshift(P.channelCfg.txChannelEn,0),1) +...
                                          bitand(bitshift(P.channelCfg.txChannelEn,-2),1);
                P.dataPath.numTxElevAnt = bitand(bitshift(P.channelCfg.txChannelEn,-1),1);
                disp('txelevant')
                disp(P.dataPath.numTxElevAnt)
            else
                fprintf('Unknown platform \n');
                return
            end
            P.channelCfg.rxChannelEn = str2num(C{2});
            P.dataPath.numRxAnt = bitand(bitshift(P.channelCfg.rxChannelEn,0),1) +...
                                  bitand(bitshift(P.channelCfg.rxChannelEn,-1),1) +...
                                  bitand(bitshift(P.channelCfg.rxChannelEn,-2),1) +...
                                  bitand(bitshift(P.channelCfg.rxChannelEn,-3),1);
            P.dataPath.numTxAnt = P.dataPath.numTxElevAnt + P.dataPath.numTxAzimAnt;

        elseif strcmp(C{1},'dataFmt')
        elseif strcmp(C{1},'profileCfg')
            P.profileCfg.startFreq = str2num(C{3});
            P.profileCfg.idleTime =  str2num(C{4});
            P.profileCfg.rampEndTime = str2num(C{6});
            P.profileCfg.freqSlopeConst = str2num(C{9});
            P.profileCfg.numAdcSamples = str2num(C{11});
            P.profileCfg.digOutSampleRate = str2num(C{12}); %uints: ksps
        elseif strcmp(C{1},'chirpCfg')
        elseif strcmp(C{1},'frameCfg')
            P.frameCfg.chirpStartIdx = str2num(C{2});
            P.frameCfg.chirpEndIdx = str2num(C{3});
            P.frameCfg.numLoops = str2num(C{4});
            P.frameCfg.numFrames = str2num(C{5});
            P.frameCfg.framePeriodicity = str2num(C{6});
        elseif strcmp(C{1},'guiMonitor')
            P.guiMonitor.detectedObjects = str2num(C{2});
            P.guiMonitor.clusters = str2num(C{3});
            P.guiMonitor.rangeAzimuthHeatMap = str2num(C{4});
            P.guiMonitor.rangeElevHeatMap = str2num(C{5});
        elseif strcmp(C{1},'dbscanCfg')
            P.dbScan.nAccFrames = str2num(C{2});
            P.dbScan.epsilon = str2num(C{3});
            P.dbScan.weight = str2num(C{4});
            P.dbScan.vFactor = str2num(C{5});
            P.dbScan.minPointsInCluster = str2num(C{6});
            P.dbScan.fixedPointScale = str2num(C{7});
        elseif strcmp(C{1},'cfarCfg')
            P.cfarCfg.detectMethod = str2num(C{2});
            P.cfarCfg.leftSkipBin = str2num(C{3});
            P.cfarCfg.closeInRangeBin = str2num(C{4});
            P.cfarCfg.searchWinSizeRange = str2num(C{5});
            P.cfarCfg.guardSizeRange = str2num(C{6});
            P.cfarCfg.searchWinSizeSpreading = str2num(C{7});
            P.cfarCfg.guardSizeSpreading = str2num(C{8});
            P.cfarCfg.rangeThresh = str2num(C{9});
            P.cfarCfg.fftSpreadingThresh = str2num(C{10});
            P.cfarCfg.noiseCalcType = str2num(C{11});
            P.cfarCfg.localPeakEnable = str2num(C{12});
            P.cfarCfg.peakAngleDiffThresh = str2num(C{13});
            P.cfarCfg.maxRangeForDetection = str2num(C{14}) * 0.1;
        end
    end
    P.dataPath.numChirpsPerFrame = (P.frameCfg.chirpEndIdx -...
                                            P.frameCfg.chirpStartIdx + 1) *...
                                            P.frameCfg.numLoops;
    P.dataPath.numDopplerBins = P.dataPath.numChirpsPerFrame / P.dataPath.numTxAnt;

    P.dataPath.rangeResolutionMeters = 3e8 * P.profileCfg.digOutSampleRate * 1e3 /...
                     (2 * P.profileCfg.freqSlopeConst * 1e12 * P.profileCfg.numAdcSamples);
    P.dataPath.numRangeBins = round(P.cfarCfg.maxRangeForDetection / P.dataPath.rangeResolutionMeters, 0);
    P.dataPath.rangeBinsPerMeter = P.dataPath.numRangeBins / P.cfarCfg.maxRangeForDetection;

    P.dataPath.rangeIdxToMeters = 3e8 * P.profileCfg.digOutSampleRate * 1e3 /...
                     (2 * P.profileCfg.freqSlopeConst * 1e12 * P.dataPath.numRangeBins);
    P.dataPath.dopplerResolutionMps = 3e8 / (2*P.profileCfg.startFreq*1e9 *...
                                        (P.profileCfg.idleTime + P.profileCfg.rampEndTime) *...
                                        1e-6 * P.dataPath.numDopplerBins * P.dataPath.numTxAnt);
    P.dataPath.maxRange = 300 * 0.9 * P.profileCfg.digOutSampleRate /(2 * P.profileCfg.freqSlopeConst * 1e3);
    P.dataPath.maxVelocity = 3e8 / (4*P.profileCfg.startFreq*1e9 *(P.profileCfg.idleTime + P.profileCfg.rampEndTime) * 1e-6 * P.dataPath.numTxAnt);

    %Calculate monitoring packet size
    tlSize = 8 %TL size 8 bytes
    TOTAL_PAYLOAD_SIZE_BYTES = 40; % size of header
    P.guiMonitor.numFigures = 1; %One figure for numerical parameers
    if P.guiMonitor.detectedObjects == 1 && P.guiMonitor.rangeAzimuthHeatMap == 1
        TOTAL_PAYLOAD_SIZE_BYTES = TOTAL_PAYLOAD_SIZE_BYTES +...
            OBJ_STRUCT_SIZE_BYTES*MAX_NUM_OBJECTS + tlSize;
        P.guiMonitor.numFigures = P.guiMonitor.numFigures + 1; %1 plots: X/Y plot
    end
    if P.guiMonitor.detectedObjects == 1 && P.guiMonitor.rangeElevHeatMap ~= 1
        TOTAL_PAYLOAD_SIZE_BYTES = TOTAL_PAYLOAD_SIZE_BYTES +...
            OBJ_STRUCT_SIZE_BYTES*MAX_NUM_OBJECTS + tlSize;
        P.guiMonitor.numFigures = P.guiMonitor.numFigures + 2; %2 plots: X/Y plot and Y/Doppler plot
    end
    if P.guiMonitor.clusters == 1
        TOTAL_PAYLOAD_SIZE_BYTES = TOTAL_PAYLOAD_SIZE_BYTES +...
            P.dataPath.numRangeBins * 2 + tlSize;
        P.guiMonitor.numFigures = P.guiMonitor.numFigures + 1;
    end
    if P.guiMonitor.rangeAzimuthHeatMap == 1
        TOTAL_PAYLOAD_SIZE_BYTES = TOTAL_PAYLOAD_SIZE_BYTES +...
            (P.dataPath.numTxAzimAnt * P.dataPath.numRxAnt) * P.dataPath.numRangeBins * 4 + tlSize;
        P.guiMonitor.numFigures = P.guiMonitor.numFigures + 1;
    end
    if P.guiMonitor.rangeElevHeatMap == 1
        TOTAL_PAYLOAD_SIZE_BYTES = TOTAL_PAYLOAD_SIZE_BYTES +...
            P.dataPath.numDopplerBins * P.dataPath.numRangeBins * 2 + tlSize;
        P.guiMonitor.numFigures = P.guiMonitor.numFigures + 1;
    end

    TOTAL_PAYLOAD_SIZE_BYTES = 32 * floor((TOTAL_PAYLOAD_SIZE_BYTES+31)/32);
    P.guiMonitor.numFigRow = 2;
    P.guiMonitor.numFigCol = ceil(P.guiMonitor.numFigures/P.guiMonitor.numFigRow);
    [P.dspFftScaleComp2D_lin, P.dspFftScaleComp2D_log] = dspFftScalCompDoppler(16, P.dataPath.numDopplerBins)
    [P.dspFftScaleComp1D_lin, P.dspFftScaleComp1D_log]  = dspFftScalCompRange(64, P.dataPath.numRangeBins)

    P.dspFftScaleCompAll_lin = P.dspFftScaleComp2D_lin * P.dspFftScaleComp1D_lin;
    P.dspFftScaleCompAll_log = P.dspFftScaleComp2D_log + P.dspFftScaleComp1D_log;

return


%Display Chirp parameters in table on screen
function hTable = displayChirpParams(Params, hTable)

    dat =  {
            'Max Range (m)', Params.dataPath.maxRange;...
            'Max Velocity (m/s)', Params.dataPath.maxVelocity;...
            'Range resolution (m)', Params.dataPath.rangeResolutionMeters;...
            'Velocity resolution (m/s)', Params.dataPath.dopplerResolutionMps;...
            'Start Frequency (Ghz)', Params.profileCfg.startFreq;...
            'Slope (MHz/us)', Params.profileCfg.freqSlopeConst;...
            'Samples per chirp', Params.profileCfg.numAdcSamples;...
            'Chirps per frame',  Params.dataPath.numChirpsPerFrame;...
            'Frame duration (ms)',  Params.frameCfg.framePeriodicity;...
            'Sampling rate (Msps)', Params.profileCfg.digOutSampleRate / 1000;...
            'Bandwidth (GHz)', Params.profileCfg.freqSlopeConst * Params.profileCfg.numAdcSamples /...
                               Params.profileCfg.digOutSampleRate;...
            'Number of Rx (MIMO)', Params.dataPath.numRxAnt;...
            'Number of Tx (MIMO)', Params.dataPath.numTxAnt;...
            'Nfft_doppler', Params.dataPath.numDopplerBins;...
            'Nfft_range', Params.dataPath.numRangeBins;};
    columnname =   {'Parameter (Units)', 'Value'};
    columnformat = {'char', 'numeric'};


    hTable.ColumnName = columnname;
    hTable.Data = dat;
    %hTable.ColumnFormat = columnformat;

    function config = readCfg(filename)
    config = cell(1,100);
    fid = fopen(filename, 'r');
    if fid == -1
        fprintf('File %s not found!\n', filename);
        return;
    else
        fprintf('Opening configuration file %s ...\n', filename);
    end
    tline = fgetl(fid);
    k=1;
    while ischar(tline)
        config{k} = tline;
        tline = fgetl(fid);
        k = k + 1;
    end
    config = config(1:k-1);
    fclose(fid);

function [y] = pow2roundup (x)
    y = 1;
    while x > y
        y = y * 2;
    end


function [] = readUartCallbackFcn(obj, event)
global bytevecAcc;
global bytevecAccLen;
global readUartFcnCntr;
global BYTES_AVAILABLE_FLAG
global BYTES_AVAILABLE_FCN_CNT
global BYTE_VEC_ACC_MAX_SIZE

bytesToRead = get(obj,'BytesAvailable');
if(bytesToRead == 0)
    return;
end

[bytevec, byteCount] = fread(obj, bytesToRead, 'uint8');

if bytevecAccLen + length(bytevec) < BYTE_VEC_ACC_MAX_SIZE * 3/4
    bytevecAcc(bytevecAccLen+1:bytevecAccLen+byteCount) = bytevec;
    bytevecAccLen = bytevecAccLen + byteCount;
else
    bytevecAccLen = 0;
end

readUartFcnCntr = readUartFcnCntr + 1;
BYTES_AVAILABLE_FLAG = 1;


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selpath = uigetdir();
handles.savePath = selpath;
disp(handles.savePath)
hTextSavePath = findobj('Tag', 'textSavePath');
hTextSavePath.String = ['Save to: ' handles.savePath];
[outstring,newpos] = textwrap(hTextSavePath,{hTextSavePath.String});
set(hTextSavePath,'String',outstring,'Position',newpos)
guidata(hObject,handles);



function editFilename_Callback(hObject, eventdata, handles)
% hObject    handle to editFilename (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editFilename as text
%        str2double(get(hObject,'String')) returns contents of editFilename as a double


% --- Executes during object creation, after setting all properties.
function editFilename_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editFilename (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function [sLin, sLog] = dspFftScalCompDoppler(fftMinSize, fftSize)
    sLin = fftMinSize/fftSize;
    sLog = 20*log10(sLin);
return

function [sLin, sLog] = dspFftScalCompRange(fftMinSize, fftSize)
    smin =  (2.^(ceil(log2(fftMinSize)./log2(4)-1)))  ./ (fftMinSize);
    sLin =  (2.^(ceil(log2(fftSize)./log2(4)-1)))  ./ (fftSize);
    sLin = sLin / smin;
    sLog = 20*log10(sLin);
return


% --- Executes on button press in pushbuttonHide.
function pushbuttonHide_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonHide (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if strcmp(handles.uipanel5.Visible,'on')
    handles.uipanel5.Visible ='off';
    handles.uipanel6.Visible ='off';
    handles.uipanel8.Visible ='off';

    hObject.String = 'Show Menu';
else
    handles.uipanel5.Visible = 'on';
    handles.uipanel6.Visible ='on';
    handles.uipanel8.Visible ='on';
    hObject.String = 'Hide Menu';
end


% --- Executes on button press in radiobuttonDetObj.
function radiobuttonDetObj_Callback(hObject, eventdata, handles)
% hObject    handle to radiobuttonDetObj (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobuttonDetObj

handles.showDetObj = get(handles.radiobuttonDetObj,  'Value');
handles.axes1.UserData.showDetObj = handles.showDetObj;


% --- Executes on button press in radiobuttonClusters.
function radiobuttonClusters_Callback(hObject, eventdata, handles)
% hObject    handle to radiobuttonClusters (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobuttonClusters

handles.showClusters = get(handles.radiobuttonClusters,  'Value');
handles.axes1.UserData.showClusters = handles.showClusters;


% --- Executes during object creation, after setting all properties.
function radiobuttonDetObj_CreateFcn(hObject, eventdata, handles)
% hObject    handle to radiobuttonDetObj(see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function radiobuttonClusters_CreateFcn(hObject, eventdata, handles)
% hObject    handle to radiobuttonClusters (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
