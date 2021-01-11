MODULE Module1
    ! Home joint
    CONST jointtarget Home:=[[0,0,0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST jointtarget End:=[[0,0,-30,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget PenHome:=[[371.669182372,0,364.737205584],[0.258819045,0,-0.965925826,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget RobotHome:=[[305.425625842,0,530],[0.5,0,0.866025404,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
   
    ! Identifier for the EGM correction.
    VAR egmident egm_id;
    ! Limits for convergance: 
    !The convergence criteria data is used to decide if the robot has reached the ordered
    !joint positions. If the difference between the ordered joint position and the actual
    !joint position is within the range of egm_minmax.min and egm_minmax.max, the
    !joint is regarded to have reached its ordered position.
    !VAR egm_minmax egm_condition := [-0.10, 0.10]; !with convergence
    VAR egm_minmax egm_condition := [-0.0, 0.0]; !without convergence
    
    ! EGM pose frames
    CONST pose egm_correction_frame := [[0, 0, 0], [1, 0, 0, 0]];
    CONST pose egm_sensor_frame     := [[0, 0, 0], [1, 0, 0, 0]];
    
    ! EGM SENSOR
    CONST string EGM_SENSOR := "EGM_REMOTE_SENSOR";
    !CONST string EGM_SENSOR := "EGM_LOCAL_SENSOR";
    !***********************************************************
    
    PROC main()
        MoveJ RobotHome,v1000,fine,tool0\WObj:=wobj0;
        !MoveL PenHome,v1000,fine,Pen_TCP\WObj:=wobj0;
        doEGMControl;
        !MoveAbsJ End,v1000,z100,tool0\WObj:=wobj0;
    ENDPROC
        
    
    PROC doEGMControl()
        ! Register an EGM id.
        EGMGetId egm_id;
        
        ! Setup the EGM communication.
        EGMSetupUC ROB_1, egm_id, "default", EGM_SENSOR, \Pose;
            
        ! Prepare for an EGM communication session.
        EGMActPose egm_id,
                   \Tool:=tool0
                   \WObj:=wobj0,
                   egm_correction_frame,
                   EGM_FRAME_BASE,
                   egm_sensor_frame,
                   EGM_FRAME_BASE
                   \X:=egm_condition
                   \Y:=egm_condition
                   \Z:=egm_condition
                   \Rx:=egm_condition
                   \Ry:=egm_condition
                   \Rz:=egm_condition
                   \SampleRate:=4
                   \MaxSpeedDeviation:=40.0;
                    
        ! Start the EGM communication session.
        EGMRunPose egm_id, EGM_STOP_HOLD, \X \Y \Z \Rx \Ry \Rz \CondTime:=5;
        
        ! Release the EGM id.
        EGMReset egm_id;
        
        ERROR
            IF ERRNO = ERR_UDPUC_COMM THEN
                TPWrite "Communication timedout";
                TRYNEXT;
            ENDIF
    ENDPROC
    
ENDMODULE