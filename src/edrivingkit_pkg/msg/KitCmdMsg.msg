uint8       cmd_esteering_mode     # 0: manual, 1: auto
float64     cmd_esteering_angle    # - 100.0 ~ +100.0
uint8       cmd_edriving_mode      # 0: manual, 1: auto
uint16      cmd_edriving_speed     # 0 ~ 3000 rpm
uint8       cmd_estop_braking      # 0: release, 1: eStop
uint8       cmd_esteering_zero_set # 0: none, 1: clear-set, 2: zero-set