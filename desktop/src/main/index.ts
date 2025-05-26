import { app, shell, BrowserWindow, ipcMain } from 'electron'
import { join } from 'path'
import { electronApp, optimizer, is } from '@electron-toolkit/utils'
import icon from '../../resources/icon.png?asset'
import { SerialPort } from 'serialport'
import { SystemState } from './constants/ipcMessages'

const createWindow = () => {
  const mainWindow = new BrowserWindow({
    width: 900,
    height: 670,
    show: false,
    autoHideMenuBar: true,
    ...(process.platform === 'linux' ? { icon } : {}),
    webPreferences: {
      preload: join(__dirname, '../preload/index.js'),
      sandbox: false
    }
  })

  mainWindow.on('ready-to-show', () => {
    mainWindow.show()
  })

  mainWindow.webContents.setWindowOpenHandler((details) => {
    shell.openExternal(details.url)
    return { action: 'deny' }
  })

  if (is.dev && process.env['ELECTRON_RENDERER_URL']) {
    mainWindow.loadURL(process.env['ELECTRON_RENDERER_URL'])
  } else {
    mainWindow.loadFile(join(__dirname, '../renderer/index.html'))
  }

  return mainWindow
}

const ODROID_VENDOR_ID = ''
const WATCH_INTERVAL = 200
const ODROID_BAUD_RATE = 115200

let currentPort: string | null = null
let currentSerial: SerialPort | null = null

const pollSerial = async (win: BrowserWindow) => {
  const ports = await SerialPort.list()

  if (currentSerial && !ports.find((port) => port.path === currentPort)) {
    currentSerial.close()
    currentSerial = null
    currentPort = null
  }

  const found = ports.find((port) => port.vendorId === ODROID_VENDOR_ID)

  if (!found) return

  currentPort = found.path
  currentSerial = new SerialPort({ path: found.path, baudRate: ODROID_BAUD_RATE })

  ipcMain.send('system-state')

  currentSerial.on('data', (data) => {
    // TODO
  })

  currentSerial.on('close', () => {
    currentPort = null
    currentSerial = null
  })

  console.log(`포트에 연결되었습니다: ${found.path}`)
}

app.whenReady().then(() => {
  electronApp.setAppUserModelId('com.electron')

  app.on('browser-window-created', (_, window) => {
    optimizer.watchWindowShortcuts(window)
  })

  const win = createWindow()

  app.on('activate', function () {
    if (BrowserWindow.getAllWindows().length === 0) createWindow()
  })

  setInterval(() => pollSerial(win), WATCH_INTERVAL)
})

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    app.quit()
  }
})

ipcMain.on('system-state', (_, data: SystemState) => {
  if (!currentSerial || !currentSerial.isOpen) return
  currentSerial.write(data, (err) => {
    if (err) {
      console.log(`시리얼 통신 중 오류 발생: ${err.message}`)
    }
  })
})
