import { MESSAGES } from '@renderer/constants/ipcMessages'
import { useTrackingActions, useTrackingState } from '@renderer/stores/systemStateStore'
import { ipcRenderer } from 'electron'
import { useEffect, useState } from 'react'
import type { Face } from '@renderer/stores/systemStateStore'

const useTrackingMode = () => {
  const state = useTrackingState()
  const actions = useTrackingActions()

  const onClickHandlerFactory = (id: number) => () => actions.setTargetFace(id)

  useEffect(() => {
    // IPC 리스너 추가
    const listener = (_, arg: Face[]) => actions.setFaces(arg)
    ipcRenderer.on(MESSAGES.IDENTIFIED_FACES, listener)

    return () => {
      ipcRenderer.removeListener(MESSAGES.IDENTIFIED_FACES, listener)
    }
  }, [])

  return {
    faces: state.faces,
    onClickHandlerFactory
  }
}

export default useTrackingMode
