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

const Header = () => {
  const { mode, actions } = useSystemState()

  return (
    <div className={styles.container}>
      {tabs.map(({ name, label }) => (
        <button
          onClick={() => actions.setMode(name)}
          className={`${styles.tab} ${mode === name && styles.active}`}
        >
          {label}
        </button>
      ))}
    </div>
  )
}

export default Header
