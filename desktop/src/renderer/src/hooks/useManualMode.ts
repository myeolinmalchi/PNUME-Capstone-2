import { ManualCommand, useManualActions } from '@renderer/stores/systemStateStore'
import { MouseEventHandler } from 'react'

const useManualMode = () => {
  const actions = useManualActions()
  const mousedownHandlerFactory =
    (command: ManualCommand): MouseEventHandler<HTMLButtonElement> =>
    () =>
      actions.setCommand(command)
  const mouseupHandler: MouseEventHandler<HTMLButtonElement> = () => actions.resetCommand()

  return {
    mouseupHandler,
    mousedownHandlerFactory
  }
}

export default useManualMode
