clc;
clear;
Left_COP_Offset = [-5.8; -13; -15.7];
Right_COP_Offset = [1.04; -16.6; -14.2];
Subject.PersonalInformation.KneeWidth = 92.0;
Subject.PersonalInformation.FootWidth = 81.5;
Subject.PersonalInformation.Hight = 1760;
Subject.PersonalInformation.Weight = 74*10;

Subject.Parameter.FPA_SENSITIVITY=0.1;
Subject.Parameter.SW_SENSITIVITY=0.8;
Subject.Parameter.DISTANCE_THRESHOLD=5625;
Subject.Parameter.FPA_ELLIPSE_PARAMETER=100;
Subject.Parameter.SW_ELLIPSE_PARAMETER=1.5625;
Subject.Parameter.FPA_MAX_THRESHOLD = 15;
Subject.Parameter.FPA_MIN_THRESHOLD = 15;
Subject.Parameter.FPA_LARGE_THRESHOLD = 5;
Subject.Parameter.FPA_SMALL_THRESHOLD = 5;
Subject.Parameter.SW_MAX_THRESHOLD = 180;
Subject.Parameter.SW_MIN_THRESHOLD = 60;
Subject.Parameter.SW_MIDDLE_MIN_THRESHOLD = 60;

SubjectID = '0627_Haihua_Subject27_SW';

STATE_STANCE = 1;
STATE_SWING = 0;
STATE_BASELINE = 0;
STATE_EXPLORATION = 1;
STATE_SELECTION = 2;
STATE_SURE = 3;
YES = 1;
NO = 0;
STATE_MAIN_INITIALIZATION = 1;
STATE_MAIN_RECORD = 2;
STATE_MAIN_FINAL = 3;
Step_Number = 1;
whichLeg = 'RT';
Sure_Number = 0;
Max_KAM=0;

flag_State_Main = STATE_MAIN_INITIALIZATION;

flag_State_Main_Initialization_Entry = YES;
flag_State_Main_Record_Entry = NO;
flag_State_Main_Final_Entry = NO;

flag_State_LeftLeg = STATE_SWING;
flag_State_LeftLeg_Stance_Entry = NO;
flag_State_LeftLeg_Swing_Entry = NO;
flag_State_RightLeg = STATE_SWING;
flag_State_RightLeg_Stance_Entry = NO;
flag_State_RightLeg_Swing_Entry = NO;
flag_State_Exploration_Done = NO;

flag_State_Section = STATE_BASELINE;

flag_State_Section_Baseline_Entry = NO;
flag_State_Section_Exploration_Entry = NO;
flag_State_Section_Selection_Entry = NO;
flag_State_Section_Sure_Entry = NO;

flag_State_Section_NormalFPA_NormalSW = NO;
flag_State_Section_ToeIn_NormalSW = NO;
flag_State_Section_ToeOut_NormalSW = NO;
%flag_State_Section_NormalFPA_SmallSW = NO;
flag_State_Section_NormalFPA_LargeSW = NO;
%flag_State_Section_ToeIn_SmallSW = NO;
flag_State_Section_ToeIn_LargeSW = NO;
%flag_State_Section_ToeOut_SmallSW = NO;
flag_State_Section_ToeOut_LargeSW = NO;


STEP_NUMBER_NORMALFPA_NORMALSW_THRESHOLD = 50;
STEP_NUMBER_TOEIN_NORMALSW_THRESHOLD = 50;
STEP_NUMBER_TOEOUT_NORMALSW_THRESHOLD = 50;
STEP_NUMBER_NORMALFPA_LARGESW_THRESHOLD = 50;
%STEP_NUMBER_NORMALFPA_SMALLSW_THRESHOLD = 0;
STEP_NUMBER_TOEIN_LARGESW_THRESHOLD = 50;
STEP_NUMBER_TOEOUT_LARGESW_THRESHOLD = 50;
%STEP_NUMBER_TOEIN_SMALLSW_THRESHOLD = 0;
%STEP_NUMBER_TOEOUT_SMALLSW_THRESHOLD = 0;

Rest_Step_Number_NormalFPA_NormalSW=STEP_NUMBER_NORMALFPA_NORMALSW_THRESHOLD;
Rest_Step_Number_ToeOut_LargeSW=STEP_NUMBER_TOEOUT_LARGESW_THRESHOLD;
Rest_Step_Number_ToeOut_NormalSW=STEP_NUMBER_TOEOUT_NORMALSW_THRESHOLD;
Rest_Step_Number_ToeIn_LargeSW=STEP_NUMBER_TOEIN_LARGESW_THRESHOLD;
Rest_Step_Number_ToeIn_NormalSW=STEP_NUMBER_TOEIN_NORMALSW_THRESHOLD;
Rest_Step_Number_NormalFPA_LargeSW=STEP_NUMBER_NORMALFPA_LARGESW_THRESHOLD;
STATE_EXPLORATION_TIME_THRESHOLD = 20;

FORCE_THRESHOLD = 20;
LEG_STATE_THRESHOLD = 2;

TIME_MAIN_INITIALIZATION = 10;
TIME_BASELINE = 10;
TIME_SURE_THRESHOLD = 10;
TIME_SELECTION_THRESHOLD = 60;

Step_Number_NormalFPA_NormalSW = 0;
Step_Number_ToeIn_NormalSW = 0;
Step_Number_ToeOut_NormalSW = 0;
Step_Number_NormalFPA_LargeSW = 0;
Step_Number_NormalFPA_SmallSW = 0;
Step_Number_ToeIn_LargeSW= 0;
Step_Number_ToeOut_LargeSW = 0;
Step_Number_ToeIn_SmallSW = 0;
Step_Number_ToeOut_SmallSW = 0;


NORMALFPA_NORMALSW = 5;
TOEIN_NORMALSW = 4;
TOEOUT_NORMALSW = 6;
NORMALFPA_LARGESW = 2;
%NORMALFPA_SMALLSW = 8;
%TOEIN_SMALLSW = 7;
TOEIN_LARGESW = 1;
TOEOUT_LARGESW = 3;
%TOEOUT_SMALLSW = 9;



% Load rotation matrix to convert markers from Vicon reference frame to anatomical reference frame
RotMat_ViconToAnatomical = create_RotMat_ViconToAnatomical('SJTU_BBL_Left');


% Initialize loop
flag_Program_Over = NO;
flag_State_Section_Basline_Entry = YES;
Right_state = STATE_SWING;
Right_Leg_StateNumber = 0;
Left_state = STATE_SWING;
Left_Leg_StateNumber = 0;
Last_FrameNum = 1;
%Initialize step number of each block
Total_FrameNumber = 0;
FPA_vals_thisStride = [];
baseline_Time=60;                                   %baseline part

ThisStep_FrameNumber = 0;

flag_State_Section_Entry = 0;
% Set filter
[Filter_b,Filter_a] = butter(2,0.2);
ThisFrame_Right_KAM_x2 = 0;
ThisFrame_Right_KAM_x1 = 0;
ThisFrame_Right_KAM_y2 = 0;
ThisFrame_Right_KAM_y1 = 0;

% Set timers
t_timer = tic;


%********************************以下为vicon关键部分**********************%
path(path,'E:\Program Files\Vicon\DataStream SDK\Win32\MATLAB');
% Load the SDK
Client.LoadViconDataStreamSDK();
% Connect as a client to the Vicon server
MyClient = Client();
MyClient.Connect('localhost:801');
pause(0.1);
if ~MyClient.IsConnected().Connected
    fprint('Connection failed!\n')
    Client.UnloadViconDataStreamSDK();
    return
end
% Enable various data types for the data stream
% MyClient.EnableSegmentData();
MyClient.EnableMarkerData();
MyClient.EnableDeviceData();
% Get the Vicon Subjectject's Name
MyClient.GetFrame();
ViconSubjectName = MyClient.GetSubjectName(1).SubjectName;
% load('VirualMarkerParameter.mat');
Time_State_Initialization_Start = toc(t_timer);
%training();
% Loop through once per Vicon frame capture
while flag_Program_Over == NO
    MyClient.GetFrame();
    FrameNum = MyClient.GetFrameNumber().FrameNumber;
    if (FrameNum == Last_FrameNum), pause(0.005); continue; end % Keep trying until a new frame is read in
    Last_FrameNum = FrameNum;
    %%
    Marker_ThisFrame_ViconRefFrame_Left_Cal = MyClient.GetMarkerGlobalTranslation(ViconSubjectName,'Left_Cal').Translation;
    Marker_ThisFrame_ViconRefFrame_Left_Mt2 = MyClient.GetMarkerGlobalTranslation(ViconSubjectName,'Left_Mt2').Translation;
    Marker_ThisFrame_ViconRefFrame_Left_Ankle_Left = MyClient.GetMarkerGlobalTranslation(ViconSubjectName,'Left_Ankle_Left').Translation;
    Marker_ThisFrame_ViconRefFrame_Left_Knee_Left = MyClient.GetMarkerGlobalTranslation(ViconSubjectName,'Left_Knee_Left').Translation;
    Marker_ThisFrame_ViconRefFrame_Right_Cal = MyClient.GetMarkerGlobalTranslation(ViconSubjectName,'Right_Cal').Translation;
    Marker_ThisFrame_ViconRefFrame_Right_Mt2 = MyClient.GetMarkerGlobalTranslation(ViconSubjectName,'Right_Mt2').Translation;
    Marker_ThisFrame_ViconRefFrame_Right_Ankle_Right = MyClient.GetMarkerGlobalTranslation(ViconSubjectName,'Right_Ankle_Right').Translation;
    Marker_ThisFrame_ViconRefFrame_Right_Knee_Right = MyClient.GetMarkerGlobalTranslation(ViconSubjectName,'Right_Knee_Right').Translation;
    Marker_ThisFrame_ViconRefFrame_Left_Knee_Right = MyClient.GetMarkerGlobalTranslation(ViconSubjectName,'Left_Knee_Right').Translation;
    Marker_ThisFrame_ViconRefFrame_Left_Ankle_Right = MyClient.GetMarkerGlobalTranslation(ViconSubjectName,'Left_Ankle_Right').Translation;
    Marker_ThisFrame_ViconRefFrame_Right_Ankle_Left = MyClient.GetMarkerGlobalTranslation(ViconSubjectName,'Right_Ankle_Left').Translation;
    Marker_ThisFrame_ViconRefFrame_Right_Knee_Left = MyClient.GetMarkerGlobalTranslation(ViconSubjectName,'Right_Knee_Left').Translation;
    
    if norm(Marker_ThisFrame_ViconRefFrame_Left_Cal) < 0.000001, fprintf('\n *** Left_Cal marker dropout on this frame ***'); continue; end % Skip to the next frame if there is dropout
    if norm(Marker_ThisFrame_ViconRefFrame_Left_Mt2) < 0.000001, fprintf('\n *** Left_Mt2 marker dropout on this frame ***'); continue; end % Skip to the next frame if there is dropout
    if norm(Marker_ThisFrame_ViconRefFrame_Left_Ankle_Left) < 0.000001, fprintf('\n *** Left_Ankle_Left marker dropout on this frame ***'); continue; end % Skip to the next frame if there is dropout
    if norm(Marker_ThisFrame_ViconRefFrame_Left_Knee_Left) < 0.000001, fprintf('\n *** Left_Knee_Left marker dropout on this frame ***'); continue; end % Skip to the next frame if there is dropout
    if norm(Marker_ThisFrame_ViconRefFrame_Right_Cal) < 0.000001, fprintf('\n *** Right_Cal marker dropout on this frame ***'); continue; end % Skip to the next frame if there is dropout
    if norm(Marker_ThisFrame_ViconRefFrame_Right_Mt2 ) < 0.000001, fprintf('\n *** Right_Mt2 marker dropout on this frame ***'); continue; end % Skip to the next frame if there is dropout
    if norm(Marker_ThisFrame_ViconRefFrame_Right_Ankle_Right) < 0.000001, fprintf('\n *** Right_Ankle_Right marker dropout on this frame ***'); continue; end % Skip to the next frame if there is dropout
    if norm(Marker_ThisFrame_ViconRefFrame_Right_Knee_Right ) < 0.000001, fprintf('\n *** Right_Knee_Right marker dropout on this frame ***'); continue; end % Skip to the next frame if there is dropout
    
    
    % Convert from Vicon reference frame to Anatomical reference frame
    ThisFrame_Marker_Left_Cal =  RotMat_ViconToAnatomical*Marker_ThisFrame_ViconRefFrame_Left_Cal ;
    ThisFrame_Marker_Left_Mt2 = RotMat_ViconToAnatomical* Marker_ThisFrame_ViconRefFrame_Left_Mt2;
    ThisFrame_Marker_Left_Ankle_Left = RotMat_ViconToAnatomical* Marker_ThisFrame_ViconRefFrame_Left_Ankle_Left;
    ThisFrame_Marker_Left_Knee_Left =  RotMat_ViconToAnatomical*Marker_ThisFrame_ViconRefFrame_Left_Knee_Left ;
    ThisFrame_Marker_Right_Cal =  RotMat_ViconToAnatomical*Marker_ThisFrame_ViconRefFrame_Right_Cal ;
    ThisFrame_Marker_Right_Mt2 = RotMat_ViconToAnatomical* Marker_ThisFrame_ViconRefFrame_Right_Mt2;
    ThisFrame_Marker_Right_Ankle_Right =  RotMat_ViconToAnatomical* Marker_ThisFrame_ViconRefFrame_Right_Ankle_Right ;
    ThisFrame_Marker_Right_Knee_Right = RotMat_ViconToAnatomical* Marker_ThisFrame_ViconRefFrame_Right_Knee_Right;
   
    
    ThisFrame_Left_FPA = Calculate_Frame_FPA(ThisFrame_Marker_Left_Mt2, ThisFrame_Marker_Left_Cal, 'LT');
    ThisFrame_Right_FPA = Calculate_Frame_FPA(ThisFrame_Marker_Right_Mt2, ThisFrame_Marker_Right_Cal, 'RT');
    
    %     if norm(Marker_ThisFrame_ViconRefFrame_Left_Knee_Right ) < 0.000001
    %         Marker_ThisFrame_ViconRefFrame_Left_Knee_Right = virtualMarker(Marker_ThisFrame_ViconRefFrame_Left_Knee_Left, ThisFrame_Left_FPA, 'KNEE','LT');
    %     end
    %     if norm(Marker_ThisFrame_ViconRefFrame_Left_Ankle_Right ) < 0.000001
    %         Marker_ThisFrame_ViconRefFrame_Left_Ankle_Right = virtualMarker(Marker_ThisFrame_ViconRefFrame_Left_Ankle_Left, ThisFrame_Left_FPA, 'ANKLE','LT');
    %     end
    %     if norm(Marker_ThisFrame_ViconRefFrame_Right_Ankle_Left ) < 0.000001
    %         Marker_ThisFrame_ViconRefFrame_Right_Ankle_Left = virtualMarker(Marker_ThisFrame_ViconRefFrame_Right_Ankle_Right, ThisFrame_Right_FPA, 'ANKLE','RT');
    %     end
    %     if norm(Marker_ThisFrame_ViconRefFrame_Right_Knee_Left ) < 0.000001
    %         Marker_ThisFrame_ViconRefFrame_Right_KNEE_Left = virtualMarker(Marker_ThisFrame_ViconRefFrame_Right_Knee_Right, ThisFrame_Right_FPA, 'KNEE','RT');
    %     end
    
    ThisFrame_Marker_Right_Ankle_Left = RotMat_ViconToAnatomical* Marker_ThisFrame_ViconRefFrame_Right_Ankle_Left;
    ThisFrame_Marker_Left_Ankle_Right =  RotMat_ViconToAnatomical* Marker_ThisFrame_ViconRefFrame_Left_Ankle_Right ;
    ThisFrame_Marker_Right_Knee_Left =  RotMat_ViconToAnatomical* Marker_ThisFrame_ViconRefFrame_Right_Knee_Left ;
    ThisFrame_Marker_Left_Knee_Right = RotMat_ViconToAnatomical* Marker_ThisFrame_ViconRefFrame_Left_Knee_Right;
    
    ThisFrame_Force_Left =  RotMat_ViconToAnatomical*-MyClient.GetGlobalForceVector(1).ForceVector;
    ThisFrame_Force_Right =  RotMat_ViconToAnatomical*-MyClient.GetGlobalForceVector(2).ForceVector;
    ThisFrame_COP_Left = RotMat_ViconToAnatomical* (MyClient.GetGlobalCentreOfPressure(1).CentreOfPressure*1000 - Left_COP_Offset);
    ThisFrame_COP_Right =  RotMat_ViconToAnatomical*(MyClient.GetGlobalCentreOfPressure(2).CentreOfPressure*1000 - Right_COP_Offset);
    
    
    if(flag_State_Main_Initialization_Entry == YES)
        flag_State_Main_Initialization_Entry = NO;
        Display_Get_Ready();
        % Get virual marker parameter
    end
    if(flag_State_Main_Record_Entry == YES)
        flag_State_Main_Record_Entry = NO;
    end
    if(flag_State_Main_Final_Entry == YES)
        flag_State_Main_Final_Entry = NO;
        Subject.Parameter.SubjectID = SubjectID;
        Subject.Parameter.WhichLeg = whichLeg;
        Subject.Result.Mean_Sure_FPA = mean(Subject.Result.Sure_FPA);
        Subject.Result.Mean_Sure_KAM = mean(Subject.Result.Sure_KAM);
        Subject.Result.Mean_Sure_SW = mean(Subject.Result.Sure_SW);
        save([SubjectID '.mat'],'Subject');
        close all;
        Plot_Matrix(MAX_MIN,Subject,Subject.Result.Candidate_KAM_Table,Subject.Result.Baseline_Right_FPA,Subject.Result.Baseline_Right_SW,Subject.Result.Baseline_Right_KAM,Subject.Result.Mean_Sure_FPA,Subject.Result.Mean_Sure_SW);
        flag_Program_Over = YES;
    end
    %% Main state transition condition
    switch flag_State_Main
        case STATE_MAIN_INITIALIZATION
            Time_State_Initialization = toc(t_timer) - Time_State_Initialization_Start;
            if Time_State_Initialization > TIME_MAIN_INITIALIZATION && abs(ThisFrame_Force_Right(3)) < FORCE_THRESHOLD
                flag_State_Main = STATE_MAIN_RECORD;
                flag_State_Main_Record_Entry = YES;
                flag_State_Section_Baseline_Entry = YES;
            end
        case STATE_MAIN_RECORD
            if(flag_State_Section == STATE_SURE)
                Time_State_Sure = toc(t_timer) - Time_State_Sure_Start;
                if Time_State_Sure > TIME_SURE_THRESHOLD
                    flag_State_Section = STATE_MAIN_FINAL;
                    flag_State_Main_Final_Entry = YES;
                end
            end
        case STATE_MAIN_FINAL
    end
    
    switch flag_State_Main
        case STATE_MAIN_INITIALIZATION
            
        case STATE_MAIN_RECORD
            %% Read data
            % Get data and calculate FPA for this frame
            
            
            [ThisFrame_Right_KAM_x0, ThisFrame_Right_ARM]= Calculate_Frame_KAM(ThisFrame_Marker_Right_Knee_Left, ThisFrame_Marker_Right_Knee_Right,ThisFrame_COP_Right, ThisFrame_Force_Right,Subject);
            ThisFrame_Right_KAM=(Filter_b(1)*ThisFrame_Right_KAM_x0+Filter_b(2)*ThisFrame_Right_KAM_x1+Filter_b(3)*ThisFrame_Right_KAM_x2- ...
                Filter_a(2)*ThisFrame_Right_KAM_y1-Filter_a(3)*ThisFrame_Right_KAM_y2)/Filter_a(1);
            ThisFrame_Right_KAM_x2 = ThisFrame_Right_KAM_x1;
            ThisFrame_Right_KAM_x1 = ThisFrame_Right_KAM_x0;
            ThisFrame_Right_KAM_y2 = ThisFrame_Right_KAM_y1;
            ThisFrame_Right_KAM_y1 = ThisFrame_Right_KAM;
            
            
            
            [ThisFrame_Left_KAM,ThisFrame_Left_ARM] = Calculate_Frame_KAM(ThisFrame_Marker_Left_Knee_Left, ThisFrame_Marker_Right_Knee_Right,ThisFrame_COP_Right, ThisFrame_Force_Right,Subject);
            ThisFrame_SW = Calculate_Frame_SW(ThisFrame_Marker_Left_Ankle_Left , ThisFrame_Marker_Left_Ankle_Right,ThisFrame_Marker_Right_Ankle_Left,ThisFrame_Marker_Right_Ankle_Right);
            
            
            Total_FrameNumber = Total_FrameNumber+1;
            ThisStep_FrameNumber = ThisStep_FrameNumber + 1;
            Subject.RawData.Timestamp(Total_FrameNumber,1) = toc(t_timer);
            
            ThisStep_Marker_Left_Cal(ThisStep_FrameNumber,:) = ThisFrame_Marker_Left_Cal;
            ThisStep_Marker_Left_Mt2(ThisStep_FrameNumber,:) = ThisFrame_Marker_Left_Mt2;
            ThisStep_Marker_Left_Ankle_Left(ThisStep_FrameNumber,:) = ThisFrame_Marker_Left_Ankle_Left;
            ThisStep_Marker_Left_Ankle_Right(ThisStep_FrameNumber,:) = ThisFrame_Marker_Left_Ankle_Right;
            ThisStep_Marker_Left_Knee_Left(ThisStep_FrameNumber,:) = ThisFrame_Marker_Left_Knee_Left;
            ThisStep_Marker_Left_Knee_Right(ThisStep_FrameNumber,:) = ThisFrame_Marker_Left_Knee_Right;
            ThisStep_Marker_Right_Cal(ThisStep_FrameNumber,:) = ThisFrame_Marker_Right_Cal;
            ThisStep_Marker_Right_Mt2(ThisStep_FrameNumber,:) = ThisFrame_Marker_Right_Mt2;
            ThisStep_Marker_Right_Ankle_Left(ThisStep_FrameNumber,:) = ThisFrame_Marker_Right_Ankle_Left;
            ThisStep_Marker_Right_Ankle_Right(ThisStep_FrameNumber,:) = ThisFrame_Marker_Right_Ankle_Right;
            ThisStep_Marker_Right_Knee_Left(ThisStep_FrameNumber,:) = ThisFrame_Marker_Right_Knee_Left;
            ThisStep_Marker_Right_Knee_Right(ThisStep_FrameNumber,:) = ThisFrame_Marker_Right_Knee_Right;
         
            ThisStep_Force_Right(ThisStep_FrameNumber,:) = ThisFrame_Force_Right;
            ThisStep_COP_Right(ThisStep_FrameNumber,:) = ThisFrame_COP_Right;
            ThisStep_Force_Left(ThisStep_FrameNumber,:) = ThisFrame_Force_Left;
            ThisStep_COP_Left(ThisStep_FrameNumber,:) = ThisFrame_COP_Left;
            ThisStep_Left_FPA(ThisStep_FrameNumber,:) = ThisFrame_Left_FPA;
            ThisStep_Right_FPA(ThisStep_FrameNumber,:) = ThisFrame_Right_FPA;
            ThisStep_Right_KAM(ThisStep_FrameNumber,:) = ThisFrame_Right_KAM;
            ThisStep_Left_KAM(ThisStep_FrameNumber,:) = ThisFrame_Left_KAM;
            ThisStep_SW(ThisStep_FrameNumber,:) = ThisFrame_SW;
            ThisStep_Right_ARM(ThisStep_FrameNumber,:)=ThisFrame_Right_ARM;
            ThisStep_Left_ARM(ThisStep_FrameNumber,:)=ThisFrame_Left_ARM;
            
            switch flag_State_LeftLeg
                case STATE_SWING
                    if abs(ThisFrame_Force_Left(3))> FORCE_THRESHOLD
                        Left_Leg_StateNumber = Left_Leg_StateNumber + 1;
                    else
                        Left_Leg_StateNumber =0;
                    end
                    if Left_Leg_StateNumber > LEG_STATE_THRESHOLD
                        flag_State_LeftLeg_Stance_Entry = YES;
                        flag_State_LeftLeg = STATE_STANCE;
                        Left_Leg_StateNumber = 0;
                    end
                case STATE_STANCE
                    if abs(ThisFrame_Force_Left(3)) < FORCE_THRESHOLD
                        Left_Leg_StateNumber = Left_Leg_StateNumber + 1;
                    else
                        Left_Leg_StateNumber =0;
                    end
                    if Left_Leg_StateNumber > LEG_STATE_THRESHOLD
                        flag_State_LeftLeg_Swing_Entry = YES;
                        flag_State_LeftLeg = STATE_SWING;
                        Left_Leg_StateNumber = 0;
                    end
            end
            %
            switch flag_State_RightLeg
                case STATE_SWING
                    if abs(ThisFrame_Force_Right(3))> FORCE_THRESHOLD
                        Right_Leg_StateNumber = Right_Leg_StateNumber + 1;
                    else
                        Right_Leg_StateNumber =0;
                    end
                    if Right_Leg_StateNumber > LEG_STATE_THRESHOLD
                        flag_State_RightLeg_Stance_Entry = YES;
                        flag_State_RightLeg = STATE_STANCE;
                        Right_Leg_StateNumber = 0;
                    end
                    
                case STATE_STANCE
                    if abs(ThisFrame_Force_Right(3)) < FORCE_THRESHOLD
                        Right_Leg_StateNumber = Right_Leg_StateNumber + 1;
                    else
                        Right_Leg_StateNumber =0;
                    end
                    if Right_Leg_StateNumber > LEG_STATE_THRESHOLD
                        flag_State_RightLeg_Swing_Entry = YES;
                        flag_State_RightLeg = STATE_SWING;
                        Right_Leg_StateNumber = 0;
                    end
            end
            if(flag_State_Section_Baseline_Entry == YES)
                Display_Baseline();
                Time_Baseline_Start = toc(t_timer);
                flag_State_Section_Baseline_Entry = NO;
            end
            
            
            % Section状态转移条件
            switch flag_State_Section
                case STATE_BASELINE
                    Time_Baseline = toc(t_timer) - Time_Baseline_Start;
                    if  Time_Baseline > TIME_BASELINE
                        if strcmp(whichLeg, 'LT')
                            Subject.Result.Baseline_Left_FPA = mean(Subject.Data.EachStep_Left_FPA(2:end,:));
                            Subject.Result.Baseline_Left_KAM = mean(Subject.Data.EachStep_Left_KAM(2:end,:));
                            Subject.Result.Baseline_Left_SW = mean(Subject.Data.EachStep_Left_SW(2:end,:));
                            MAX_MIN.MAX_FPA = Subject.Result.Baseline_Left_FPA + Subject.Parameter.FPA_MAX_THRESHOLD;
                            MAX_MIN.MIN_FPA = Subject.Result.Baseline_Left_FPA - Subject.Parameter.FPA_MIN_THRESHOLD;
                            MAX_MIN.Large_FPA = Subject.Result.Baseline_Left_FPA + Subject.Parameter.FPA_LARGE_THRESHOLD;
                            MAX_MIN.Small_FPA = Subject.Result.Baseline_Left_FPA - Subject.Parameter.FPA_SMALL_THRESHOLD;
                            MAX_MIN.MAX_SW = Subject.Result.Baseline_Left_SW + Subject.Parameter.SW_MAX_THRESHOLD;
                            MAX_MIN.MIN_SW = Subject.Result.Baseline_Left_SW - Subject.Parameter.SW_MIN_THRESHOLD;
                            MAX_MIN.Middle_SW = Subject.Result.Baseline_Left_SW + Subject.Parameter.SW_MIDDLE_MIN_THRESHOLD;
                            %MAX_MIN.Small_SW = Subject.Result.Baseline_Left_SW;
                            TOEOUT_THRESHOLD = MAX_MIN.Large_FPA;
                            TOEIN_THRESHOLD = MAX_MIN.Small_FPA;
                            MIDDLE_SW_THRESHOLD = MAX_MIN.Middle_SW;
                            %SMALL_SW_THRESHOLD = MAX_MIN.Small_SW;
                            REDUCE_GOAL = 0.85*Subject.Result.Baseline_Left_KAM;
                        else
                            Subject.Result.Baseline_Right_FPA = mean(Subject.Data.EachStep_Right_FPA(2:end,:));
                            Subject.Result.Baseline_Right_KAM = mean(Subject.Data.EachStep_Right_KAM(2:end,:));
                            Subject.Result.Baseline_Right_SW = mean(Subject.Data.EachStep_Right_SW(2:end,:));
                            MAX_MIN.MAX_FPA = Subject.Result.Baseline_Right_FPA + Subject.Parameter.FPA_MAX_THRESHOLD;
                            MAX_MIN.MIN_FPA = Subject.Result.Baseline_Right_FPA - Subject.Parameter.FPA_MIN_THRESHOLD;
                            MAX_MIN.Large_FPA = Subject.Result.Baseline_Right_FPA + Subject.Parameter.FPA_LARGE_THRESHOLD;
                            MAX_MIN.Small_FPA = Subject.Result.Baseline_Right_FPA - Subject.Parameter.FPA_SMALL_THRESHOLD;
                            MAX_MIN.MAX_SW = Subject.Result.Baseline_Right_SW + Subject.Parameter.SW_MAX_THRESHOLD;
                            MAX_MIN.MIN_SW = Subject.Result.Baseline_Right_SW - Subject.Parameter.SW_MIN_THRESHOLD;
                            MAX_MIN.Middle_SW = Subject.Result.Baseline_Right_SW + Subject.Parameter.SW_MIDDLE_MIN_THRESHOLD;
                            %MAX_MIN.Small_SW = Subject.Result.Baseline_Right_SW;
                            TOEOUT_THRESHOLD = MAX_MIN.Large_FPA;
                            TOEIN_THRESHOLD = MAX_MIN.Small_FPA;
                            MIDDLE_SW_THRESHOLD = MAX_MIN.Middle_SW;
                            %SMALL_SW_THRESHOLD = MAX_MIN.Small_SW;
                            REDUCE_GOAL = 0.85*Subject.Result.Baseline_Right_KAM;
                        end
                        flag_State_Section_Exploration_Entry = YES;
                        flag_State_Section = STATE_EXPLORATION;
                    end
                case STATE_EXPLORATION
                    if flag_State_Exploration_Done == YES
                        flag_State_Exploration_Done = NO;
                        flag_State_Section_Selection_Entry = YES;
                        flag_State_Section = STATE_SELECTION;
                        Time_State_Selection_Start = toc(t_timer);
                    end
                    
                case STATE_SELECTION
                    Time_State_Selection = toc(t_timer) - Time_State_Selection_Start;
                    if Time_State_Selection > TIME_SELECTION_THRESHOLD
                        flag_State_Section = STATE_SURE;
                        flag_State_Section_Sure_Entry = YES;
                        Time_State_Sure_Start = toc(t_timer);
                    end
                case STATE_SURE
                otherwise
            end
            if(flag_State_Section_Exploration_Entry == YES)
                flag_State_Section_Exploration_Entry = NO;
                time_State_Exploration_Start = toc(t_timer);
            end
            ThisStep_State_LeftLeg_Stance_Entry(ThisStep_FrameNumber,1) = flag_State_LeftLeg_Stance_Entry;
            ThisStep_State_LeftLeg_Swing_Entry(ThisStep_FrameNumber,1) = flag_State_LeftLeg_Swing_Entry;
            ThisStep_State_LeftLeg(ThisStep_FrameNumber,1) = flag_State_LeftLeg;
            ThisStep_State_RightLeg_Stance_Entry(ThisStep_FrameNumber,1) = flag_State_RightLeg_Stance_Entry;
            ThisStep_State_RightLeg_Swing_Entry(ThisStep_FrameNumber,1) = flag_State_RightLeg_Swing_Entry;
            ThisStep_State_RightLeg(ThisStep_FrameNumber,1) = flag_State_RightLeg;
            ThisStep_State_Section(ThisStep_FrameNumber,1) = flag_State_Section;
            
            %% 左脚状态转移程序
            if(flag_State_LeftLeg_Stance_Entry == YES)
                flag_State_LeftLeg_Stance_Entry = NO;
            end
            
            if(flag_State_LeftLeg_Swing_Entry == YES)
                if strcmp(whichLeg, 'LT')
                    Step_Number = Step_Number+1;
                    Subject.Data.EachStep_Left_FPA(Step_Number,1) = Calculate_Step_FPA(ThisStep_Left_FPA, ThisStep_State_LeftLeg);
                    Subject.Data.EachStep_Left_KAM(Step_Number,1) = Calculate_Step_KAM(ThisStep_Left_KAM);
                    Subject.Data.EachStep_Left_SW(Step_Number,1) = Calculate_Step_SW(ThisStep_SW,ThisStep_State_LeftLeg,ThisStep_State_RightLeg,ThisStep_State_LeftLeg_Stance_Entry);
                    Subject.Data.EachStep_Left_ARM(Step_Number,3) = Calculate_Step_ARM(ThisStep_Left_KAM, ThisStep_Left_ARM);
                    if flag_State_Section == STATE_EXPLORATION
                        Update_Step(Subject.Data.EachStep_Right_FPA(Step_Number,1),Subject.Data.EachStep_Right_SW(Step_Number,1),MAX_MIN);
                        if Subject.Data.EachStep_Left_FPA(Step_Number,1) > TOEOUT_THRESHOLD
                            if Subject.Data.EachStep_Left_SW(Step_Number,1) > MIDDLE_SW_THRESHOLD
                                Step_Number_ToeOut_LargeSW = Step_Number_ToeOut_LargeSW+1;
                                Rest_Step_Number_ToeOut_LargeSW=STEP_NUMBER_TOEOUT_LARGESW_THRESHOLD-Step_Number_ToeOut_LargeSW;
                            else
                                Step_Number_ToeOut_NormalSW = Step_Number_ToeOut_NormalSW+1;
                                Rest_Step_Number_ToeOut_NormalSW=STEP_NUMBER_TOEOUT_NORMALSW_THRESHOLD-Step_Number_ToeOut_NormalSW;
                            end
                        elseif Subject.Data.EachStep_Left_FPA(Step_Number,1) < TOEIN_THRESHOLD
                            if Subject.Data.EachStep_Left_SW(Step_Number,1) > MIDDLE_SW_THRESHOLD
                                Step_Number_ToeIn_LargeSW = Step_Number_ToeIn_LargeSW+1;
                                Rest_Step_Number_ToeIn_LargeSW=STEP_NUMBER_TOEIN_LARGESW_THRESHOLD-Step_Number_ToeIn_LargeSW;
                            else
                                Step_Number_ToeIn_NormalSW = Step_Number_ToeIn_NormalSW+1;
                                Rest_Step_Number_ToeIn_NormalSW=STEP_NUMBER_TOEIN_NORMALSW_THRESHOLD-Step_Number_ToeIn_NormalSW;
                            end
                        else
                            if Subject.Data.EachStep_Left_SW(Step_Number,1) > MIDDLE_SW_THRESHOLD
                                Step_Number_NormalFPA_LargeSW = Step_Number_NormalFPA_LargeSW+1;
                                Rest_Step_Number_NormalFPA_LargeSW=STEP_NUMBER_NORMALFPA_LARGESW_THRESHOLD-Step_Number_NormalFPA_LargeSW;
                            else
                                Step_Number_NormalFPA_NormalSW = Step_Number_NormalFPA_NormalSW+1;
                                Rest_Step_Number_NormalFPA_NormalSW=STEP_NUMBER_NORMALFPA_NORMALSW_THRESHOLD-Step_Number_NormalFPA_NormalSW;
                            end
                        end
                    elseif flag_State_Section == STATE_SURE
                        Sure_Number = Sure_Number+1;
                        Subject.Result.Sure_FPA(Sure_Number,1) =Subject.Data.EachStep_Right_FPA(Step_Number,1);
                        Subject.Result.Sure_SW(Sure_Number,1) =Subject.Data.EachStep_Right_SW(Step_Number,1);
                        Subject.Result.Sure_KAM(Sure_Number,1) =Subject.Data.EachStep_Right_KAM(Step_Number,1);
                    elseif flag_State_Section == STATE_SELECTION
                        Update_Step(Subject.Data.EachStep_Left_FPA(Step_Number,1), Subject.Data.EachStep_Left_SW(Step_Number,1),MAX_MIN);
                    end
                    switch flag_State_Exploration_Section
                        case NORMALFPA_NORMALSW
                            Update_Section(Rest_Step_Number_NormalFPA_NormalSW);
                        case TOEIN_NORMALSW
                            Update_Section(Rest_Step_Number_ToeIn_NormalSW);
                        case TOEOUT_NORMALSW
                            Update_Section(Rest_Step_Number_ToeOut_NormalSW);
                        case NORMALFPA_LARGESW
                            Update_Section(Rest_Step_Number_NormalFPA_LargeSW);
                        case TOEIN_LARGESW
                            Update_Section(Rest_Step_Number_ToeIn_LargeSW);
                        case TOEOUT_LARGESW
                            Update_Section(Rest_Step_Number_ToeOut_LargeSW);
                    end
                    ThisStep_FrameNumber = 0;
                    ThisStep_Force_Right = [];
                    ThisStep_COP_Right = [];
                    ThisStep_Force_Left = [];
                    ThisStep_COP_Left = [];
                    ThisStep_Marker_Left_Cal = [];
                    ThisStep_Marker_Left_Mt2 = [];
                    ThisStep_Marker_Left_Foot_Middle = [];
                    ThisStep_Marker_Left_Ankle_Left = [];
                    ThisStep_Marker_Left_Ankle_Right = [];
                    ThisStep_Marker_Left_Leg_Middle = [];
                    ThisStep_Marker_Left_Knee_Left= [];
                    ThisStep_Marker_Left_Knee_Right = [];
                    ThisStep_Marker_Right_Cal = [];
                    ThisStep_Marker_Right_Mt2= [];
                    ThisStep_Marker_Right_Foot_Middle = [];
                    ThisStep_Marker_Right_Ankle_Left= [];
                    ThisStep_Marker_Right_Ankle_Right = [];
                    ThisStep_Marker_Right_Leg_Middle= [];
                    ThisStep_Marker_Right_Knee_Left= [];
                    ThisStep_Marker_Right_Knee_Right = [];
                    ThisStep_Left_FPA = [];
                    ThisStep_Right_FPA = [];
                    ThisStep_Left_KAM = [];
                    ThisStep_Right_KAM = [];
                    ThisStep_SW = [];
                    ThisStep_State_LeftLeg = [];
                    ThisStep_State_LeftLeg_Stance_Entry = [];
                    ThisStep_State_LeftLeg_Swing_Entry = [];
                    ThisStep_State_RightLeg = [];
                    ThisStep_State_RightLeg_Stance_Entry = [];
                    ThisStep_State_RightLeg_Swing_Entry = [];
                    ThisStep_State_Section = [];
                end
                flag_State_Leg_Swing_Entry = NO;
            end
            %% 右脚状态转移程序
            if(flag_State_RightLeg_Stance_Entry == YES)
                flag_State_RightLeg_Stance_Entry = NO;
            end
            
            if(flag_State_RightLeg_Swing_Entry == YES)
                if strcmp(whichLeg, 'RT')
                    Step_Number = Step_Number+1;
                    Subject.Data.EachStep_Right_FPA(Step_Number,1) = Calculate_Step_FPA(ThisStep_Right_FPA, ThisStep_State_RightLeg);
                    Subject.Data.EachStep_Right_KAM(Step_Number,1) = Calculate_Step_KAM(ThisStep_Right_KAM);
                    Subject.Data.EachStep_Right_SW(Step_Number,1) = Calculate_Step_SW(ThisStep_SW,ThisStep_State_LeftLeg,ThisStep_State_RightLeg,ThisStep_State_RightLeg_Stance_Entry);
                    Subject.Data.EachStep_Right_ARM(Step_Number,:) = Calculate_Step_ARM(ThisStep_Right_KAM, ThisStep_Right_ARM);
                    if flag_State_Section == STATE_EXPLORATION
                        Update_Step(Subject.Data.EachStep_Right_FPA(Step_Number,1),Subject.Data.EachStep_Right_SW(Step_Number,1),MAX_MIN);
                        if Subject.Data.EachStep_Right_FPA(Step_Number,1) > TOEOUT_THRESHOLD && Subject.Data.EachStep_Right_FPA(Step_Number,1) < MAX_MIN.MAX_FPA
                            if Subject.Data.EachStep_Right_SW(Step_Number,1) > MIDDLE_SW_THRESHOLD && Subject.Data.EachStep_Right_SW(Step_Number,1) < MAX_MIN.MAX_SW
                                Step_Number_ToeOut_LargeSW = Step_Number_ToeOut_LargeSW+1;
                                Rest_Step_Number_ToeOut_LargeSW=STEP_NUMBER_TOEOUT_LARGESW_THRESHOLD-Step_Number_ToeOut_LargeSW;
                            else
                                Step_Number_ToeOut_NormalSW = Step_Number_ToeOut_NormalSW+1;
                                Rest_Step_Number_ToeOut_NormalSW=STEP_NUMBER_TOEOUT_NORMALSW_THRESHOLD-Step_Number_ToeOut_NormalSW;
                            end
                        elseif Subject.Data.EachStep_Right_FPA(Step_Number,1) < TOEIN_THRESHOLD && Subject.Data.EachStep_Right_FPA(Step_Number,1) > MAX_MIN.MIN_FPA
                            if Subject.Data.EachStep_Right_SW(Step_Number,1) > MIDDLE_SW_THRESHOLD  && Subject.Data.EachStep_Right_SW(Step_Number,1) < MAX_MIN.MAX_SW 
                                Step_Number_ToeIn_LargeSW = Step_Number_ToeIn_LargeSW+1;
                                Rest_Step_Number_ToeIn_LargeSW=STEP_NUMBER_TOEIN_LARGESW_THRESHOLD-Step_Number_ToeIn_LargeSW;
                            else
                                Step_Number_ToeIn_NormalSW = Step_Number_ToeIn_NormalSW+1;
                                Rest_Step_Number_ToeIn_NormalSW=STEP_NUMBER_TOEIN_NORMALSW_THRESHOLD-Step_Number_ToeIn_NormalSW;
                            end
                        else
                            if Subject.Data.EachStep_Right_SW(Step_Number,1) > MIDDLE_SW_THRESHOLD&& Subject.Data.EachStep_Right_SW(Step_Number,1) < MAX_MIN.MAX_SW
                                Step_Number_NormalFPA_LargeSW = Step_Number_NormalFPA_LargeSW+1;
                                Rest_Step_Number_NormalFPA_LargeSW=STEP_NUMBER_NORMALFPA_LARGESW_THRESHOLD-Step_Number_NormalFPA_LargeSW;
                            else
                                Step_Number_NormalFPA_NormalSW = Step_Number_NormalFPA_NormalSW+1;
                                Rest_Step_Number_NormalFPA_NormalSW=STEP_NUMBER_NORMALFPA_NORMALSW_THRESHOLD-Step_Number_NormalFPA_NormalSW;
                            end
                        end
                        switch flag_State_Exploration_Section
                            case NORMALFPA_NORMALSW
                                Update_Section(Rest_Step_Number_NormalFPA_NormalSW);
                            case TOEIN_NORMALSW
                                Update_Section(Rest_Step_Number_ToeIn_NormalSW);
                            case TOEOUT_NORMALSW
                                Update_Section(Rest_Step_Number_ToeOut_NormalSW);
                            case NORMALFPA_LARGESW
                                Update_Section(Rest_Step_Number_NormalFPA_LargeSW);
                            case TOEIN_LARGESW
                                Update_Section(Rest_Step_Number_ToeIn_LargeSW);
                            case TOEOUT_LARGESW
                                Update_Section(Rest_Step_Number_ToeOut_LargeSW);
                        end
                    elseif flag_State_Section == STATE_SURE
                        Sure_Number = Sure_Number+1;
                        Subject.Result.Sure_FPA(Sure_Number,1) =Subject.Data.EachStep_Right_FPA(Step_Number,1);
                        Subject.Result.Sure_SW(Sure_Number,1) =Subject.Data.EachStep_Right_SW(Step_Number,1);
                        Subject.Result.Sure_KAM(Sure_Number,1) =Subject.Data.EachStep_Right_KAM(Step_Number,1);
                    elseif flag_State_Section == STATE_SELECTION
                        Update_Step(Subject.Data.EachStep_Right_FPA(Step_Number,1), Subject.Data.EachStep_Right_SW(Step_Number,1),MAX_MIN);
                    end
                    ThisStep_FrameNumber = 0;
                    ThisStep_Force_Right = [];
                    ThisStep_COP_Right = [];
                    ThisStep_Force_Left = [];
                    ThisStep_COP_Left = [];
                    ThisStep_Marker_Left_Cal = [];
                    ThisStep_Marker_Left_Mt2 = [];
                    ThisStep_Marker_Left_Foot_Middle = [];
                    ThisStep_Marker_Left_Ankle_Left = [];
                    ThisStep_Marker_Left_Ankle_Right = [];
                    ThisStep_Marker_Left_Leg_Middle = [];
                    ThisStep_Marker_Left_Knee_Left= [];
                    ThisStep_Marker_Left_Knee_Right = [];
                    ThisStep_Marker_Right_Cal = [];
                    ThisStep_Marker_Right_Mt2= [];
                    ThisStep_Marker_Right_Foot_Middle = [];
                    ThisStep_Marker_Right_Ankle_Left= [];
                    ThisStep_Marker_Right_Ankle_Right = [];
                    ThisStep_Marker_Right_Leg_Middle= [];
                    ThisStep_Marker_Right_Knee_Left= [];
                    ThisStep_Marker_Right_Knee_Right = [];
                    ThisStep_Left_FPA = [];
                    ThisStep_Right_FPA = [];
                    ThisStep_Left_KAM = [];
                    ThisStep_Right_KAM = [];
                    
                    ThisStep_SW = [];
                    ThisStep_State_LeftLeg = [];
                    ThisStep_State_LeftLeg_Stance_Entry = [];
                    ThisStep_State_LeftLeg_Swing_Entry = [];
                    ThisStep_State_RightLeg = [];
                    ThisStep_State_RightLeg_Stance_Entry = [];
                    ThisStep_State_RightLeg_Swing_Entry = [];
                    ThisStep_State_Section = [];
                end
                flag_State_RightLeg_Swing_Entry = NO;
            end
            % Section 状态转移程序
            if(flag_State_Section_Selection_Entry == YES)
                flag_State_Section_Selection_Entry = NO;
                [Subject.Result.Candidate_KAM_Table, Selection_Result_Table] = Cal_Exploration(Subject.Data.EachStep_Right_FPA, Subject.Data.EachStep_Right_SW, Subject.Data.EachStep_Right_KAM, MAX_MIN,REDUCE_GOAL,Subject);
                Display_Selection(Selection_Result_Table, MAX_MIN,Subject);
                Time_State_Selection_Start = toc(t_timer);
            end
            if(flag_State_Section_Sure_Entry == YES)
                Display_Sure();
                flag_State_Section_Sure_Entry = NO;
            end
            
            if(flag_State_LeftLeg == STATE_SWING)
            else
            end
            switch flag_State_LeftLeg
                case STATE_SWING
                    
                case STATE_STANCE
                    
                otherwise
                    
            end
            
            switch flag_State_Section
                case STATE_BASELINE
                    
                case STATE_EXPLORATION
                    time_State_Exploration = toc(t_timer) - time_State_Exploration_Start;
                    
                    if Step_Number_NormalFPA_NormalSW < STEP_NUMBER_NORMALFPA_NORMALSW_THRESHOLD
                        if flag_State_Section_NormalFPA_NormalSW == NO
                            flag_State_Section_Entry = NORMALFPA_NORMALSW;
                            flag_State_Exploration_Section = NORMALFPA_NORMALSW;
                            time_State_Exploration_Start = toc(t_timer);
                            flag_State_Section_NormalFPA_NormalSW = YES;
                        else
                            flag_State_Section_Entry = NO;
                        end
                    elseif Step_Number_ToeIn_NormalSW < STEP_NUMBER_TOEIN_NORMALSW_THRESHOLD  %% toe in, normal SW
                        if flag_State_Section_ToeIn_NormalSW == NO
                            flag_State_Section_Entry = TOEIN_NORMALSW;
                            flag_State_Exploration_Section = TOEIN_NORMALSW;
                            time_State_Exploration_Start = toc(t_timer);
                            flag_State_Section_ToeIn_NormalSW = 1;
                        else
                            flag_State_Section_Entry = NO;
                        end
                    elseif Step_Number_ToeOut_NormalSW < STEP_NUMBER_TOEOUT_NORMALSW_THRESHOLD %% toe out, normal SW
                        if flag_State_Section_ToeOut_NormalSW == 0
                            flag_State_Section_Entry = TOEOUT_NORMALSW;
                            flag_State_Exploration_Section = TOEOUT_NORMALSW;
                            time_State_Exploration_Start = toc(t_timer);
                            flag_State_Section_ToeOut_NormalSW = 1;
                        else
                            flag_State_Section_Entry = NO;
                        end
                    elseif Step_Number_NormalFPA_LargeSW < STEP_NUMBER_NORMALFPA_LARGESW_THRESHOLD%% normal FPA, large SW
                        if flag_State_Section_NormalFPA_LargeSW == 0
                            flag_State_Section_Entry = NORMALFPA_LARGESW;
                            flag_State_Exploration_Section = NORMALFPA_LARGESW;
                            time_State_Exploration_Start = toc(t_timer);
                            flag_State_Section_NormalFPA_LargeSW = 1;
                        else
                            flag_State_Section_Entry = NO;
                        end
                        %                     elseif Step_Number_NormalFPA_SmallSW < STEP_NUMBER_NORMALFPA_SMALLSW_THRESHOLD%% normal FPA, small SW
                        %                         if flag_State_Section_NormalFPA_SmallSW == 0
                        %                             flag_State_Section_Entry = NORMALFPA_SMALLSW;
                        %                             time_State_Exploration_Start = toc(t_timer);
                        %                             flag_State_Section_NormalFPA_SmallSW = 1;
                        %                         else
                        %                             flag_State_Section_Entry = NO;
                        %                         end
                    elseif Step_Number_ToeIn_LargeSW < STEP_NUMBER_TOEIN_LARGESW_THRESHOLD%% toe in, large SW
                        if flag_State_Section_ToeIn_LargeSW == 0
                            flag_State_Section_Entry = TOEIN_LARGESW;
                            flag_State_Exploration_Section = TOEIN_LARGESW;
                            time_State_Exploration_Start = toc(t_timer);
                            flag_State_Section_ToeIn_LargeSW = 1;
                        else
                            flag_State_Section_Entry = NO;
                        end
                    elseif Step_Number_ToeOut_LargeSW < STEP_NUMBER_TOEOUT_LARGESW_THRESHOLD%% toe out, large SW
                        if flag_State_Section_ToeOut_LargeSW == 0
                            flag_State_Section_Entry = TOEOUT_LARGESW;
                            flag_State_Exploration_Section = TOEOUT_LARGESW;
                            time_State_Exploration_Start = toc(t_timer);
                            flag_State_Section_ToeOut_LargeSW = 1;
                        else
                            flag_State_Section_Entry = NO;
                        end
                    else
                        Display_Calculating();
                        flag_State_Exploration_Done = YES;
                    end
                    switch flag_State_Section_Entry
                        case NORMALFPA_NORMALSW
                            Display_Exploration(NORMALFPA_NORMALSW, MAX_MIN);
                        case TOEIN_NORMALSW
                            Display_Exploration(TOEIN_NORMALSW, MAX_MIN);
                        case TOEOUT_NORMALSW
                            Display_Exploration(TOEOUT_NORMALSW, MAX_MIN);
                        case NORMALFPA_LARGESW
                            Display_Exploration(NORMALFPA_LARGESW, MAX_MIN);
                        case TOEIN_LARGESW
                            Display_Exploration(TOEIN_LARGESW, MAX_MIN);
                        case TOEOUT_LARGESW
                            Display_Exploration(TOEOUT_LARGESW, MAX_MIN);
                    end
                    
            end
            %%
            Subject.RawData.Marker_Left_Cal(Total_FrameNumber,:) =  ThisFrame_Marker_Left_Cal;
            Subject.RawData.Marker_Left_Mt2(Total_FrameNumber,:) = ThisFrame_Marker_Left_Mt2;
            Subject.RawData.Marker_Left_Ankle_Left(Total_FrameNumber,:) = ThisFrame_Marker_Left_Ankle_Left;
            Subject.RawData.Marker_Left_Ankle_Right(Total_FrameNumber,:) =  ThisFrame_Marker_Left_Ankle_Right;
            Subject.RawData.Marker_Left_Knee_Left(Total_FrameNumber,:) =  ThisFrame_Marker_Left_Knee_Left;
            Subject.RawData.Marker_Left_Knee_Right(Total_FrameNumber,:) = ThisFrame_Marker_Left_Knee_Right;
            Subject.RawData.Marker_Right_Cal(Total_FrameNumber,:) =  ThisFrame_Marker_Right_Cal;
            Subject.RawData.Marker_Right_Mt2(Total_FrameNumber,:) = ThisFrame_Marker_Right_Mt2;
            Subject.RawData.Marker_Right_Ankle_Left(Total_FrameNumber,:) = ThisFrame_Marker_Right_Ankle_Left;
            Subject.RawData.Marker_Right_Ankle_Right(Total_FrameNumber,:) =  ThisFrame_Marker_Right_Ankle_Right;
            Subject.RawData.Marker_Right_Knee_Left(Total_FrameNumber,:) =  ThisFrame_Marker_Right_Knee_Left;
            Subject.RawData.Marker_Right_Knee_Right(Total_FrameNumber,:) = ThisFrame_Marker_Right_Knee_Right;
            Subject.RawData.Force_Right(Total_FrameNumber,:) = ThisFrame_Force_Right;
            Subject.RawData.COP_Right(Total_FrameNumber,:) = ThisFrame_COP_Right;
            Subject.RawData.Force_Left(Total_FrameNumber,:) = ThisFrame_Force_Left;
            Subject.RawData.COP_Left(Total_FrameNumber,:) = ThisFrame_COP_Left;
            
            Subject.RawData.ARM_Right(Total_FrameNumber,:) = ThisFrame_Right_ARM;
            Subject.RawData.ARM_Left(Total_FrameNumber,:) = ThisFrame_Left_ARM;
            Subject.Data.Left_FPA(Total_FrameNumber,1) = ThisFrame_Left_FPA;
            Subject.Data.Step_Number(Total_FrameNumber,1) = Step_Number;
            Subject.Data.Left_FPA(Total_FrameNumber,1) = ThisFrame_Left_FPA;
            Subject.Data.Right_FPA(Total_FrameNumber,1) = ThisFrame_Right_FPA;
            Subject.Data.Right_KAM(Total_FrameNumber,1) = ThisFrame_Right_KAM;
            Subject.Data.Left_KAM(Total_FrameNumber,1) = ThisFrame_Left_KAM;
            Subject.Data.SW(Total_FrameNumber,1) = ThisFrame_SW;
            Subject.Data.State_LeftLeg_Swing_Entry(Total_FrameNumber,1) = flag_State_LeftLeg_Swing_Entry;
            Subject.Data.State_LeftLeg(Total_FrameNumber,1) = flag_State_LeftLeg;
            Subject.Data.State_RightLeg_Stance_Entry(Total_FrameNumber,1) = flag_State_RightLeg_Stance_Entry;
            Subject.Data.State_RightLeg_Swing_Entry(Total_FrameNumber,1) = flag_State_RightLeg_Swing_Entry;
            Subject.Data.State_RightLeg(Total_FrameNumber,1) = flag_State_RightLeg;
            Subject.Data.State_Section(Total_FrameNumber,1) = flag_State_Section;
            Subject.Data.State_LeftLeg_Stance_Entry(Total_FrameNumber,1) = flag_State_LeftLeg_Stance_Entry;
    end
end