import { SystemMode, useSystemState } from '@renderer/stores/systemStateStore'
import styles from './index.module.css'

const tabs: {
  name: SystemMode
  label: string
}[] = [
  { name: 'manual', label: 'Manual Mode' },
  { name: 'tracking', label: 'Tracking Mode' },
  { name: 'paused', label: 'Paused' }
]

// 공통 헤더
const Header = () => {
  const { mode, actions } = useSystemState()

  const setModeHandler = (newMode: SystemMode) => () => {
    // mode가 null일 경우
    // usb 연결 전이므로 실행 방지
    if (mode === null) {
      alert('장치가 인식되지 않았습니다.')
      return
    }

    actions.setMode(newMode)
  }

  return (
    <div className={styles.container}>
      {tabs.map(({ name, label }) => (
        <button
          key={name}
          onClick={setModeHandler(name)}
          className={`${styles.tab} ${mode === name && styles.active}`}
        >
          {label}
        </button>
      ))}
    </div>
  )
}

export default Header
