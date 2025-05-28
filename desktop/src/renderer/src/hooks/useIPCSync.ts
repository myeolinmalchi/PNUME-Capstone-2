import { MESSAGES } from '@renderer/constants/ipcMessages'
import { SystemState, useSystemState } from '@renderer/stores/systemStateStore'
import { useEffect } from 'react'

// IPC 통신을 통한 Client-Server간 데이터 동기화 hook
const useIPCSync = () => {
  const { mode, trackingState, manualState, actions } = useSystemState()

  useEffect(() => {
    if (!mode) return

    window.electron.ipcRenderer.send(MESSAGES.SYSTEM_STATE, {
      mode,
      trackingState,
      manualState
    })
  }, [mode, trackingState, manualState])

  useEffect(() => {
    const systemStateHandler = (_, arg: SystemState) => actions.setSystemState(arg)
    const initHandler = () => actions.setMode('paused')

    window.electron.ipcRenderer.on(MESSAGES.SYSTEM_STATE, systemStateHandler)
    window.electron.ipcRenderer.on(MESSAGES.INIT_SYSTEM, initHandler)

    return () => {
      window.electron.ipcRenderer.removeListener(MESSAGES.SYSTEM_STATE, systemStateHandler)
      window.electron.ipcRenderer.removeListener(MESSAGES.INIT_SYSTEM, initHandler)
    }
  }, [])
}

export default useIPCSync
