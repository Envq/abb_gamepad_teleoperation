MODULE TRob1Main
!======================================================================================================
! Copyright (c) 2018, ABB Schweiz AG
! All rights reserved.
!
! Redistribution and use in source and binary forms, with
! or without modification, are permitted provided that
! the following conditions are met:
!
!    * Redistributions of source code must retain the
!      above copyright notice, this list of conditions
!      and the following disclaimer.
!    * Redistributions in binary form must reproduce the
!      above copyright notice, this list of conditions
!      and the following disclaimer in the documentation
!      and/or other materials provided with the
!      distribution.
!    * Neither the name of ABB nor the names of its
!      contributors may be used to endorse or promote
!      products derived from this software without
!      specific prior wrSitten permission.
!
! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
! AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
! IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
! ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
! LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
! DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
! SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
! CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
! OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
! THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
!======================================================================================================
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
        !doEGMJointControl;
        doEGMPoseControl;
        MoveAbsJ End, v1000, fine, tool0\WObj:=wobj0;
    ENDPROC
        
    PROC doEGMJointControl()
        ! Register an EGM id.
        EGMGetId egm_id;
        
        ! Setup the EGM communication.
        EGMSetupUC ROB_1, egm_id, "default", EGM_SENSOR \Joint;
        
        ! Prepare for an EGM communication session. (sampleRate in ms)
        EGMActJoint egm_id
                    \J1:=egm_condition
                    \J2:=egm_condition
                    \J3:=egm_condition
                    \J4:=egm_condition
                    \J5:=egm_condition
                    \J6:=egm_condition
                    \SampleRate:=4
                    \MaxSpeedDeviation:=20.0;
                
        ! Start the EGM communication session.
        EGMRunJoint egm_id, EGM_STOP_HOLD, \J1 \J2 \J3 \J4 \J5 \J6 \CondTime:=1;
        
        ! Release the EGM id.
        EGMReset egm_id;
        
        ERROR
            IF ERRNO = ERR_UDPUC_COMM THEN
                TPWrite "Communication timedout";
                TRYNEXT;
            ENDIF
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