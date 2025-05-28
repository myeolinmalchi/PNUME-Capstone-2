import arrow from '@renderer/assets/arrow-top.png'
import styles from './index.module.css'
import useManualMode from '@renderer/hooks/useManualMode'

// 수동 조작 모드
const ManualMode = () => {
  const { mouseupHandler, mousedownHandlerFactory } = useManualMode()

  return (
    <div className={styles.container}>
      {/** 상하좌우 조작 */}
      <div className={styles['plane-container']}>
        <button onMouseDown={mousedownHandlerFactory('move_up')} onMouseUp={mouseupHandler}>
          <img src={arrow} className={`${styles.arrow} ${styles.up}`} />
        </button>
        <button onMouseDown={mousedownHandlerFactory('turn_left')} onMouseUp={mouseupHandler}>
          <img src={arrow} className={`${styles.arrow} ${styles.left}`} />
        </button>
        <button onMouseDown={mousedownHandlerFactory('turn_right')} onMouseUp={mouseupHandler}>
          <img src={arrow} className={`${styles.arrow} ${styles.right}`} />
        </button>
        <button onMouseDown={mousedownHandlerFactory('move_down')} onMouseUp={mouseupHandler}>
          <img src={arrow} className={`${styles.arrow} ${styles.down}`} />
        </button>
      </div>

      {/** 앞뒤 조작 */}
      <div className={styles['horizontal-container']}>
        <button onMouseDown={mousedownHandlerFactory('move_forward')} onMouseUp={mouseupHandler}>
          <img src={arrow} className={`${styles.arrow} ${styles.down}`} />
        </button>
        <button onMouseDown={mousedownHandlerFactory('move_backward')} onMouseUp={mouseupHandler}>
          <img src={arrow} className={`${styles.arrow} ${styles.up}`} />
        </button>
      </div>
    </div>
  )
}

export default ManualMode
