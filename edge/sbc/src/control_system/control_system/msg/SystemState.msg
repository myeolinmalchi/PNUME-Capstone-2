#####################################
##  System Mode
######################################
string IDLE     = "IDLE"
string TRACKING = "TRACKING"
string MANUAL   = "MANUAL"

######################################
##  Command (manual mode only)
######################################
string NONE        = "NONE"
string TURN_LEFT   = "TURN_LEFT"
string TURN_RIGHT  = "TURN_RIGHT"
string MOVE_UP     = "MOVE_UP"
string MOVE_DOWN   = "MOVE_DOWN"
string MOVE_FRONT  = "MOVE_FRONT"
string MOVE_BACK   = "MOVE_BACK"

######################################
##  Field
######################################
string mode          # 현재 모드
string command       # 수행 중인 명령
int32 tracking_id    # 타겟 얼굴 ID