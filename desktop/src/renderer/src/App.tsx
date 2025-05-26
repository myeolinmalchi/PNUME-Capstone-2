import Header from './components/Header'
import ManualMode from './components/modes/ManualMode'
import './styles/reset.css'

import styles from './App.module.css'
import { useSystemState } from './stores/systemStateStore'
import PausedMode from './components/modes/PausedMode'

const App = () => {
  const { mode } = useSystemState()

  return (
    <>
      <Header />
      <div className={styles.container}>
        {mode === null && (
          <PausedMode
            text="장치가 인식되지 않았습니다.
USB-C 케이블을 통해 장치와 연결하세요."
          />
        )}
        {mode === 'manual' && <ManualMode />}
        {mode === 'paused' && (
          <PausedMode
            text="장치를 조작하려면 Tracking
또는 Manual 모드로 전환하세요."
          />
        )}
      </div>
    </>
  )
}

export default App
