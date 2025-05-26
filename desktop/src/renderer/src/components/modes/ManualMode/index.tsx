import arrow from '@renderer/assets/arrow-top.png'
import styles from './index.module.css'
import useManualMode from '@renderer/hooks/useManualMode'

const ManualMode = () => {
  const { mouseupHandler, mousedownHandlerFactory } = useManualMode()

  return (
    <div className={styles.container}>
      <div className={styles['plane-container']}>
        <button onMouseDown={mousedownHandlerFactory('move_up')}>
          <img src={arrow} className={`${styles.arrow} ${styles.up}`} />
        </button>
        <button onMouseDown={mousedownHandlerFactory('turn_left')}>
          <img src={arrow} className={`${styles.arrow} ${styles.left}`} />
        </button>
        <button onMouseDown={mousedownHandlerFactory('turn_right')}>
          <img src={arrow} className={`${styles.arrow} ${styles.right}`} />
        </button>
        <button onMouseDown={mousedownHandlerFactory('move_down')}>
          <img src={arrow} className={`${styles.arrow} ${styles.down}`} />
        </button>
      </div>
      <div className={styles['horizontal-container']}>
        <button onMouseDown={mousedownHandlerFactory('move_forward')}>
          <img src={arrow} className={`${styles.arrow} ${styles.down}`} />
        </button>
        <button onMouseDown={mousedownHandlerFactory('move_backward')}>
          <img src={arrow} className={`${styles.arrow} ${styles.up}`} />
        </button>
      </div>
    </div>
  )
}

export default ManualMode
