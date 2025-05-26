import { MESSAGES } from '@renderer/constants/ipcMessages'
import { SystemState, useSystemState } from '@renderer/stores/systemStateStore'
import { ipcRenderer } from 'electron'
import { useEffect } from 'react'

// IPC 통신을 통한 장치간 데이터 동기화 hook
const useIPCSync = () => {
  const { mode, trackingState, manualState, actions } = useSystemState()

  useEffect(() => {
    ipcRenderer.send(MESSAGES.SYSTEM_STATE, {
      mode,
      trackingState,
      manualState
    })
  }, [mode, trackingState, manualState])

  useEffect(() => {
    // IPC 리스너 추가
    const listenter = (_, arg: SystemState) => actions.setSystemState(arg)
    ipcRenderer.on(MESSAGES.SYSTEM_STATE, listenter)

    return () => {
      ipcRenderer.removeListener(MESSAGES.SYSTEM_STATE, listenter)
    }
  }, [])
}

export default useIPCSync
