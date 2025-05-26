import styles from './index.module.css'

type PausedProps = {
  text: string
}

const PausedMode = ({ text }: PausedProps) => {
  return (
    <>
      <span className={styles.text}>{text}</span>
    </>
  )
}

export default PausedMode
