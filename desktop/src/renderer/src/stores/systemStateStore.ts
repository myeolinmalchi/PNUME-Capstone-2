import { create } from 'zustand'
import { BASELINE_LIMITS } from '@renderer/constants/tracking'

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

type SystemStateAction = {
  actions: {
    setMode: (mode: SystemMode) => void
    setSystemState: (systemState: SystemState) => void
    trackingActions: {
      setBaseline: (width?: number, height?: number) => void
      setTargetFace: (faceId: number) => void
      setFaces: (faces: Face[]) => void
    }

    manualActions: {
      resetCommand: () => void
      setCommand: (command: ManualCommand) => void
    }
  }
}

// USB-OTG 초기 연결시에
// System State 및 기타 설정값 모두 받아옴
const initialState: SystemState = {
  mode: null,
  trackingState: {
    baseline: {
      width: 60,
      height: 60
    },
    currentId: null,
    faces: []
  },
  manualState: {
    currentCommand: null
  }
}

export const useSystemState = create<SystemState & SystemStateAction>()((set) => ({
  ...initialState,
  actions: {
    setMode: (mode: SystemMode) => set({ mode }),
    setSystemState: (systemState) => set(systemState),
    trackingActions: {
      setBaseline: (width?: number, height?: number) => {
        const test =
          (!width && !height) ||
          (width && width < BASELINE_LIMITS.WIDTH.MIN) ||
          (width && width > BASELINE_LIMITS.WIDTH.MAX) ||
          (height && height < BASELINE_LIMITS.HEIGHT.MIN) ||
          (height && height > BASELINE_LIMITS.HEIGHT.MAX)

        if (test) return

        set((prev) => ({
          trackingState: {
            ...prev.trackingState,
            baseline: {
              width: width ?? prev.trackingState.baseline.width,
              height: height ?? prev.trackingState.baseline.height
            }
          }
        }))
      },
      setTargetFace: (faceId: number) =>
        set((prev) => ({
          trackingState: {
            ...prev.trackingState,
            currentId: faceId
          }
        })),
      setFaces: (faces) =>
        set((prev) => ({
          trackingState: {
            ...prev.trackingState,
            faces
          }
        }))
    },
    manualActions: {
      resetCommand: () => set(() => ({ manualState: { currentCommand: null } })),
      setCommand: (command: ManualCommand) =>
        set(() => ({ manualState: { currentCommand: command } }))
    }
  }
}))

const useManualState = () => useSystemState((state) => state.manualState)
const useManualActions = () => useSystemState((state) => state.actions.manualActions)

const useTrackingState = () => useSystemState((state) => state.trackingState)
const useTrackingActions = () => useSystemState((state) => state.actions.trackingActions)

export { useManualState, useManualActions, useTrackingState, useTrackingActions }
export type { SystemMode, ManualCommand, Face, SystemState }
