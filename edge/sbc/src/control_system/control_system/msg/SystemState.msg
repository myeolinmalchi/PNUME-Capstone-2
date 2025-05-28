#####################################
##  System Mode
######################################
string IDLE     = "paused"
string TRACKING = "tracking"
string MANUAL   = "manual"

######################################
##  Command (manual mode only)
######################################
string NONE        = "none"
string TURN_LEFT   = "turn_left"
string TURN_RIGHT  = "turn_right"
string MOVE_UP     = "move_up"
string MOVE_DOWN   = "move_down"
string MOVE_FRONT  = "move_front"
string MOVE_BACK   = "move_back"

######################################
##  Field
######################################
string mode          # 현재 모드
string command       # 수행 중인 명령
int32 tracking_id    # 타겟 얼굴 ID