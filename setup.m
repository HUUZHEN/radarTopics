function varargout = setup(varargin)
% setup - GUI 的主要函數，負責初始化並啟動 GUI。
% varargin - 可變長度的輸入參數。
% varargout - 可變長度的輸出參數。

% 設定 GUI 為單例模式 (只允許開啟一個實例)
gui_Singleton = 1;

% 建立 GUI 狀態結構體 (struct)，用來存放 GUI 的相關函數和屬性
gui_State = struct('gui_Name',       mfilename, ... % 設定 GUI 名稱為目前的 M 檔案名稱
                   'gui_Singleton',  gui_Singleton, ... % 設定 GUI 是否為單例模式
                   'gui_OpeningFcn', @setup_OpeningFcn, ... % 設定 GUI 初始化函數
                   'gui_OutputFcn',  @setup_OutputFcn, ... % 設定 GUI 輸出函數
                   'gui_LayoutFcn',  [] , ... % 佈局函數 (預設為空，使用 GUIDE 預設佈局)
                   'gui_Callback',   []); % 回呼函數 (預設為空，稍後可能會指定)

% 檢查是否有輸入參數，且第一個參數是否為字串 (回呼函數名稱)
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1}); % 轉換字串為函數句柄
end

% 檢查是否有輸出變數 (nargout 為 MATLAB 內建變數，表示輸出變數個數)
if nargout
    % 若有輸出，則回傳 gui_mainfcn 的輸出值
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    % 若沒有輸出，則直接執行 gui_mainfcn
    gui_mainfcn(gui_State, varargin{:});
end


function setup_OpeningFcn(hObject, eventdata, handles, varargin)
% setup_OpeningFcn - GUI 初始化函數。
% hObject - 當前 GUI 的句柄。
% eventdata - 事件數據 (預設不使用)。
% handles - 存放 GUI 內所有對象的結構體。
% varargin - 可變長度的輸入參數。

% 設定 GUI 的輸出為當前 GUI 物件。
handles.output = hObject;

% 初始化變數。
handles.camIndex = -1; % 相機索引，預設為 -1 (未指定)
handles.hDataSerialPort = []; % 用於數據傳輸的串列埠，預設為空。
handles.hControlSerialPort = []; % 用於控制的串列埠，預設為空。

% 設定房間邊界 (單位可能為米)
handles.wall = struct('left', -6, 'right', 6, 'front', 6, 'back', 0);

% 初始化角度變數。
handles.angle = 0;

% 設定預設的雷達配置。
handles.cfg = struct('filename', 'mmw_pplcount_demo_default.cfg', 'loaded', 0);

% 初始化子區域變數。
handles.subzone = [];

% 儲存更新後的 handles 結構。
guidata(hObject, handles);

% 初始化 GUI 設定。
initialize_gui(hObject, handles, false);

% 繪製雷達房間。
drawRadarRoom(handles);

% 重新獲取更新後的 handles 結構。
handles = guidata(hObject);

% 查找顯示 COM 連接狀態的 UI 元件。
hCOMStatus = findobj('Tag', 'textCOMStatus');

% 根據串列埠的狀態更新顯示文字。
if(~isempty(handles.hControlSerialPort) && ~isempty(handles.hDataSerialPort))
    update = '狀態: 已連接';
else
    update = '狀態: 未連接';
end

% 更新 UI 顯示 COM 連接狀態。
set(hCOMStatus,'String', update); 

% 暫停 GUI，等待使用者操作。
uiwait(handles.figure1);

ffunction varargout = setup_OutputFcn(hObject, eventdata, handles)
% setup_OutputFcn - GUI 的輸出函數。
% hObject - 當前 GUI 的句柄。
% eventdata - 事件數據 (預設不使用)。
% handles - 存放 GUI 內所有對象的結構體。

% 設定 GUI 的輸出變數。
varargout{1} = handles; % 將 handles 結構體作為輸出返回。



function btnStart_Callback(hObject, eventdata, handles)

if(length(instrfind('Type','serial', 'Status','open'))>=2 && ~isempty(handles.hControlSerialPort) && ~isempty(handles.hDataSerialPort))
    mmwDemoCliPrompt = char('mmwDemo:/>');
    hControlSerialPort = handles.hControlSerialPort;

    
    cliCfg = readCfg(handles.cfg.filename);
    [Params cliCfg] = parseCfg(cliCfg, handles.angle); 
    handles.params = Params;
    guidata(hObject, handles);
    
    fprintf('Sending configuration from %s file to IWR16xx ...\n', handles.cfg.filename);
    disp(hControlSerialPort);
    for k=1:length(cliCfg)
        fprintf(hControlSerialPort, cliCfg{k});
        fprintf('%s\n', cliCfg{k});
        echo = fgetl(hControlSerialPort); % Get an echo of a command
        done = hControlSerialPort 
        prompt = fread(hControlSerialPort, size(mmwDemoCliPrompt,2)); % Get the prompt back 
    end
    fclose(hControlSerialPort);
    delete(hControlSerialPort);
    
    
    editLW_Callback(findobj('Tag', 'editLW'), eventdata, handles);
    editRW_Callback(findobj('Tag', 'editRW'), eventdata, handles);
    editFW_Callback(findobj('Tag', 'editFW'), eventdata, handles);
    editBW_Callback(findobj('Tag', 'editBW'), eventdata, handles);
   
    setup_OutputFcn(hObject,eventdata,guidata(hObject));
    uiresume(gcbf);
else
    warndlg('請確認Comport狀態');
end
    
function btnCancel_Callback(hObject, eventdata, handles)

delete(instrfind('Type','serial', 'Status','open'))
initialize_gui(gcbf, handles, true);
close(gcbf)

function initialize_gui(fig_handle, handles, isreset)

guidata(handles.figure1, handles);

function pushbuttonBrowse_Callback(hObject, eventdata, handles)
selectFile = findobj('Tag','radiobuttonSelectFile');
if(selectFile.Value)
     [filename, pathname] = ...
     uigetfile('*.cfg','*.*');
     configurationFileName = [pathname filename];
     handles.cfg.filename = configurationFileName;
     if (filename ~= 0)
        % Read Chirp Configuration file
        cliCfg = readCfg(handles.cfg.filename);
        [Params cliCfg] = parseCfg(cliCfg, handles.angle); 
        handles.params = Params;
        guidata(hObject,handles)
        drawRadarRoom(handles);
     end
     guidata(hObject,handles)
end

function popupUART_Callback(hObject, eventdata, handles)
selectIndx = hObject.Value;
menuStrings = hObject.String;
strPort = menuStrings{selectIndx};
comNum = str2num(strPort(4:end));
hObject.UserData = struct('strPort', strPort, 'comNum', comNum);
function popupUART_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
[strPorts numPorts] = get_com_ports();
hObject.String = strPorts{:,1};
hObject.UserData = struct('strPort', strPorts{1,1}, 'comNum', numPorts(1,1));


function popupData_Callback(hObject, eventdata, handles)
selectIndx = hObject.Value;
menuStrings = hObject.String;
strPort = menuStrings{selectIndx};
comNum = str2num(strPort(4:end));
hObject.UserData = struct('strPort', strPort, 'comNum', comNum);



function popupData_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
[strPorts numPorts] = get_com_ports();
hObject.String = strPorts{:,2};
hObject.UserData = struct('strPort', strPorts{1,2}, 'comNum', numPorts(1,2));
function btnConnect_Callback(hObject, eventdata, handles)

hUARTPort = findobj('Tag','editUART');
hDataPort = findobj('Tag','editDATA');

controlSerialPort = 'COM4';%hUARTPort.UserData.strPort;
dataSerialPort = 'COM5';%hDataPort.UserData.strPort;

if ~isempty(instrfind('Type','serial'))
    disp('Serial port(s) already open. Re-initializing...');
    delete(instrfind('Type','serial'));  % delete open serial ports.
end
hDataSerialPort = configureDataSport(dataSerialPort, 65536);
hControlSerialPort = configureControlPort(controlSerialPort);
handles.hDataSerialPort = hDataSerialPort;
handles.hControlSerialPort = hControlSerialPort;
hCOMStatus = findobj('Tag', 'textCOMStatus');
update = '狀態: 已連接';
set(hCOMStatus,'String', update); 
guidata(hObject,handles);


function editLW_Callback(hObject, eventdata, handles)
handles.wall.left = -1*str2double(get(hObject,'String'));
guidata(hObject,handles);
drawRadarRoom(handles);
h = guidata(hObject);
fprintf('left wall: %d \n', h.wall.left)


function editLW_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

handles.wall.left = -1*str2double(get(hObject,'String'));
guidata(hObject,handles);




function editRW_Callback(hObject, eventdata, handles)
handles.wall.right = str2double(get(hObject,'String'));
guidata(hObject,handles);
drawRadarRoom(handles);
h = guidata(hObject);
fprintf('R wall: %d \n', h.wall.right)

function editRW_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

handles.wall.right = str2double(get(hObject,'String'));
guidata(hObject,handles);



function editBW_Callback(hObject, eventdata, handles)

handles.wall.back = -1*str2double(get(hObject,'String'));
guidata(hObject,handles);
drawRadarRoom(handles);
h = guidata(hObject);
fprintf('back wall: %d \n', h.wall.back)


function editBW_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

handles.wall.back = -1*str2double(get(hObject,'String'));
guidata(hObject,handles);



function editFW_Callback(hObject, eventdata, handles)
% hObject    handle to editFW (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editFW as text
%        str2double(get(hObject,'String')) returns contents of editFW as a double
handles.wall.front = str2double(get(hObject,'String'));
guidata(hObject,handles);
drawRadarRoom(handles);
h = guidata(hObject);
fprintf('front wall: %d \n', h.wall.front)


% --- Executes during object creation, after setting all properties.
function editFW_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

handles.wall.front = str2double(get(hObject,'String'));
guidata(hObject,handles);


function axes1_CreateFcn(hObject, eventdata, handles)

imshow('images/setupfig.jpg','Parent',handles.axes1);
function radiobuttonSelectFile_Callback(hObject, eventdata, handles)

function radiobuttonUseDefault_Callback(hObject, eventdata, handles)

if(hObject.Value)
     % Get Chirp Config File
     handles.cfg.filename = 'mmw_pplcount_demo_default.cfg';
     guidata(hObject,handles)
end

function [P, cliCfg] = parseCfg(cliCfg, azimuthTilt)
    P=[];
    for k=1:length(cliCfg)
        C = strsplit(cliCfg{k});
        if strcmp(C{1},'channelCfg')
            P.channelCfg.txChannelEn = str2double(C{3});
            P.dataPath.numTxAzimAnt = bitand(bitshift(P.channelCfg.txChannelEn,0),1) +...
                                      bitand(bitshift(P.channelCfg.txChannelEn,-1),1);
            P.dataPath.numTxElevAnt = 0;
            P.channelCfg.rxChannelEn = str2double(C{2});
            P.dataPath.numRxAnt = bitand(bitshift(P.channelCfg.rxChannelEn,0),1) +...
                                  bitand(bitshift(P.channelCfg.rxChannelEn,-1),1) +...
                                  bitand(bitshift(P.channelCfg.rxChannelEn,-2),1) +...
                                  bitand(bitshift(P.channelCfg.rxChannelEn,-3),1);
            P.dataPath.numTxAnt = P.dataPath.numTxElevAnt + P.dataPath.numTxAzimAnt;
                                
        elseif strcmp(C{1},'trackingCfg')
            % error check for azimuth tilt
            % if cfg tilt is not same as GUI specified tilt; GUI specified
            % tilt is used instead
            if((str2num(C{8})-90)*pi/180 ~= azimuthTilt)
                temp = cliCfg{k};
                temp(end+1-length(C{8}):end) = '';
                ang = num2str(90-azimuthTilt);
                temp = [temp ang];
                cliCfg{k} = temp;
                fprintf('trackingCfg specifies %d.\n', (str2num(C{8})-90)*pi/180 );
                fprintf('GUI specifies %d. %s will be used for azimuth in cfg.\n',azimuthTilt, ang);
            end
        elseif strcmp(C{1},'profileCfg')
            P.profileCfg.startFreq = str2double(C{3});
            P.profileCfg.idleTime =  str2double(C{4});
            P.profileCfg.rampEndTime = str2double(C{6});
            P.profileCfg.freqSlopeConst = str2double(C{9});
            P.profileCfg.numAdcSamples = str2double(C{11});
            P.profileCfg.digOutSampleRate = str2double(C{12}); %uints: ksps
        elseif strcmp(C{1},'chirpCfg')
        elseif strcmp(C{1},'frameCfg')
            P.frameCfg.chirpStartIdx = str2double(C{2});
            P.frameCfg.chirpEndIdx = str2double(C{3});
            P.frameCfg.numLoops = str2double(C{4});
            P.frameCfg.numFrames = str2double(C{5});
            P.frameCfg.framePeriodicity = str2double(C{6});
        elseif strcmp(C{1},'guiMonitor')
            P.guiMonitor.detectedObjects = str2double(C{2});
            P.guiMonitor.logMagRange = str2double(C{3});
            P.guiMonitor.rangeAzimuthHeatMap = str2double(C{4});
            P.guiMonitor.rangeDopplerHeatMap = str2double(C{5});
        end
    end
    P.dataPath.numChirpsPerFrame = (P.frameCfg.chirpEndIdx -...
                                            P.frameCfg.chirpStartIdx + 1) *...
                                            P.frameCfg.numLoops;
    P.dataPath.numDopplerBins = P.dataPath.numChirpsPerFrame / P.dataPath.numTxAnt;
    P.dataPath.numRangeBins = pow2roundup(P.profileCfg.numAdcSamples);
    P.dataPath.rangeResolutionMeters = 3e8 * P.profileCfg.digOutSampleRate * 1e3 /...
                     (2 * P.profileCfg.freqSlopeConst * 1e12 * P.profileCfg.numAdcSamples);
    P.dataPath.rangeIdxToMeters = 3e8 * P.profileCfg.digOutSampleRate * 1e3 /...
                     (2 * P.profileCfg.freqSlopeConst * 1e12 * P.dataPath.numRangeBins);
    P.dataPath.dopplerResolutionMps = 3e8 / (2*P.profileCfg.startFreq*1e9 *...
                                        (P.profileCfg.idleTime + P.profileCfg.rampEndTime) *...
                                        1e-6 * P.dataPath.numDopplerBins * P.dataPath.numTxAnt);

function [y] = pow2roundup (x)
    y = 1;
    while x > y
        y = y * 2;
    end
    
function [sphandle] = configureDataSport(comPortString, bufferSize)
  
    %comPortString = ['COM' num2str(comPortNum)];
    sphandle = serial(comPortString,'BaudRate',921600);
    set(sphandle,'Terminator', '');
    set(sphandle,'InputBufferSize', bufferSize);
    set(sphandle,'Timeout',10);
    set(sphandle,'ErrorFcn',@dispError);
    fopen(sphandle);

function [sphandle] = configureControlPort(comPortString)
  
    sphandle = serial(comPortString,'BaudRate',115200);
    set(sphandle,'Parity','none')    
    set(sphandle,'Terminator','LF')        
    fopen(sphandle);

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


function figure1_CloseRequestFcn(hObject, eventdata, handles)
delete(hObject);


function popupWebcam_Callback(hObject, eventdata, handles)

function popupWebcam_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
hObject.String = webcamlist();

guidata(hObject,handles);

function [strPorts numPorts] = get_com_ports()
    
    
    command = 'wmic path win32_pnpentity get caption /format:list | find "COM"';
    [status, cmdout] = system (command);
    UART_COM = regexp(cmdout, 'UART\s+\(COM[0-9]+', 'match');
    UART_COM = (regexp(UART_COM, 'COM[0-9]+', 'match'));
    DATA_COM = regexp(cmdout, 'Data\s+Port\s+\(COM[0-9]+', 'match');
    DATA_COM = (regexp(DATA_COM, 'COM[0-9]+', 'match'));
    
    n = length(UART_COM);
    if (n==0)
        errordlg('Error: No Device Detected')
        return
    else
        CLI_PORT = zeros(n,1);
        S_PORT = zeros(n,1);
        strPorts = {};
        for i=1:n
            temp = cell2mat(UART_COM{1,i});
            strPorts{i,1} = temp;
            CLI_PORT(i,1) = str2num(temp(4:end));
            temp = cell2mat(DATA_COM{1,i});
            strPorts{i,2} = temp;
            S_PORT(i,1) = str2num(temp(4:end));
        end

        CLI_PORT = sort(CLI_PORT);
        S_PORT = sort(S_PORT);
        numPorts = [CLI_PORT, S_PORT];
    end



function editBox_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function drawRadarRoom(handles)
ax = handles.axes1;
if ishandle(ax)
    children = get(ax, 'children');
    delete(children);
    scene.azimuthTilt = handles.angle*pi/180;
    wall = handles.wall;
    %sensor parameters
    if(isfield(handles, 'params'))
        sensor.rangeMax = floor(handles.params.dataPath.numRangeBins*handles.params.dataPath.rangeResolutionMeters);
    else 
        sensor.rangeMax = 6;
    end
    sensor.rangeMin = 1;
    sensor.azimuthFoV = 120*pi/180; 
    sensor.angles = linspace(-sensor.azimuthFoV/2, sensor.azimuthFoV/2, 128);
    %hold(ax, 'on')
    plot(ax, sensor.rangeMin*sin(sensor.angles+scene.azimuthTilt), sensor.rangeMin*cos(sensor.angles+scene.azimuthTilt), '-r'); 
    plot(ax, [0 sensor.rangeMax*sin(sensor.angles+scene.azimuthTilt) 0],[0 sensor.rangeMax*cos(sensor.angles+scene.azimuthTilt) 0], '-r');
    scene.areaBox = [wall.left wall.back abs(wall.left)+wall.right wall.front+abs(wall.back)];
    
    margin = 0.5; %[m]
    scene.maxPos = [scene.areaBox(1)-margin ...
                    scene.areaBox(1)+scene.areaBox(3)+margin ...
                    scene.areaBox(2)-margin ...
                    scene.areaBox(2)+scene.areaBox(4)+margin];
    ax.DataAspectRatio = [1 1 1];
    axis(ax, scene.maxPos);
    ax.CameraUpVector = [0,-1, 0];
    grid(ax, 'on');
    grid(ax, 'minor');               

    rectangle(ax, 'Position', scene.areaBox, 'EdgeColor','k', 'LineStyle', '-', 'LineWidth', 2);
end

colors='brgcm';
numBoxes = size(handles.subzone,1);
for nBoxes = 1:numBoxes
    hTargetBoxHandle(nBoxes)= rectangle('Parent', ax, 'Position', handles.subzone(nBoxes,:), 'EdgeColor', colors(nBoxes), 'LineWidth', 4);
end



function editUART_Callback(hObject, eventdata, handles)

strPort = get(hObject, 'String');
comNum = str2double(strPort(4:end));
hObject.UserData = struct('strPort', strPort, 'comNum', comNum);

function editUART_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
[strPorts numPorts] = get_com_ports();
hObject.String = strPorts{:,1};
hObject.UserData = struct('strPort', strPorts{1,1}, 'comNum', numPorts(1,1))



function editDATA_Callback(hObject, eventdata, handles)
strPort = get(hObject, 'String');
comNum = str2double(strPort(4:end));
hObject.UserData = struct('strPort', strPort, 'comNum', comNum);



function editDATA_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

[strPorts numPorts] = get_com_ports();
hObject.String = strPorts{:,2};
hObject.UserData = struct('strPort', strPorts{1,2}, 'comNum', numPorts(1,2));


% --- Executes during object creation, after setting all properties.
function text17_CreateFcn(hObject, eventdata, handles)
% hObject    handle to text17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
