uint8       fd_esteering_mode       # 0: manual, 1: auto
float64     fd_esteeringle_angle    # - 100.0 ~ +100.0
uint8       fd_estop_braking_mode   # 0: release, 1: eStop
float64     fd_estop_duty           # -99.000 ~ 99.000

######################## For VCU #############################

uint8       fd_edriving_mode        # 0: manual, 1: auto
float64     fd_edriving_speed       # 0 ~ 3000.00 rpm
uint8       fd_vcu_status           # 0: None, 1: Init_state, 2: Operating_state, 3: abnormal_state
uint8       fd_vcu_mode             # 0: None, 1: Manual, 2: auto, 3: reserved, 4: e-stop
uint8       fd_vcu_alv              # 0 ~ 255   