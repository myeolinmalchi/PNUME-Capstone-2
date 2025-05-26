type SystemMode = 'tracking' | 'manual' | 'paused' | 'disabled'

type ManualCommand =
  | 'turn_left'
  | 'turn_right'
  | 'move_up'
  | 'move_down'
  | 'move_forward'
  | 'move_backward'

type Face = {
  bbox: {
    x1: number
    y1: number
    x2: number
    y2: number
  }
  id: number
}

type SystemState = {
  // 시스템 모드
  mode: SystemMode | null

  // Tracking 모드 관련 상태
  trackingState: {
    baseline: {
      width: number
      height: number
    }
    currentId: number | null
    faces: Face[]
  }

  // Tracking 모드 관련 상태
  manualState: {
    currentCommand: ManualCommand | null
  }
}

export type { SystemMode, ManualCommand, Face, SystemState }
