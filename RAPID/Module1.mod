MODULE Module1
    ! Home joint
    CONST jointtarget Home:=[[0,0,0,0,30,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST jointtarget End:=[[0,0,-90,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    ! Identifier for the EGM correction.
    VAR egmident egm_id;
    ! Limits for convergance: 
    !The convergence criteria data is used to decide if the robot has reached the ordered
    !joint positions. If the difference between the ordered joint position and the actual
    !joint position is within the range of egm_minmax.min and egm_minmax.max, the
    !joint is regarded to have reached its ordered position.
    VAR egm_minmax egm_condition := [-0.1, 0.1];
    
    ! EGM pose frames.
    CONST pose egm_correction_frame := [[0, 0, 0], [1, 0, 0, 0]];
    CONST pose egm_sensor_frame     := [[0, 0, 0], [1, 0, 0, 0]];
    
    ! EGM SENSOR
    CONST string EGM_SENSOR := "EGM_REMOTE_SENSOR";
    !CONST string EGM_SENSOR := "EGM_LOCAL_SENSOR";
    !***********************************************************
    
    PROC main()
        MoveAbsJ Home, v1000, fine, tool0\WObj:=wobj0;
        doEGMPoseControl;
        !MoveAbsJ End, v1000, fine, tool0\WObj:=wobj0;
    ENDPROC
        
    
    PROC doEGMPoseControl()
        ! Register an EGM id.
        EGMGetId egm_id;
        
        ! Setup the EGM communication.
        EGMSetupUC ROB_1, egm_id, "default", EGM_SENSOR, \Pose;
            
        ! Prepare for an EGM communication session.
        EGMActPose egm_id,
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
                   \MaxSpeedDeviation:=20.0;
                    
        ! Start the EGM communication session.
        EGMRunPose egm_id, EGM_STOP_RAMP_DOWN, \X \Y \Z \Rx \Ry \Rz \CondTime:=5 \RampOutTime:=5;
        
        ! Release the EGM id.
        EGMReset egm_id;
        
        ERROR
            IF ERRNO = ERR_UDPUC_COMM THEN
                TPWrite "Communication timedout";
                TRYNEXT;
            ENDIF
    ENDPROC
    
ENDMODULE