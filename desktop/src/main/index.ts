import { app, shell, BrowserWindow, ipcMain } from 'electron'
import { join } from 'path'
import { electronApp, optimizer, is } from '@electron-toolkit/utils'
import { SerialPort } from 'serialport'
import { SystemState } from './constants/ipcMessages'

const createWindow = () => {
  const mainWindow = new BrowserWindow({
    width: 900,
    height: 670,
    show: false,
    autoHideMenuBar: true,
    webPreferences: {
      preload: join(__dirname, '../preload/index.js'),
      sandbox: false
    },
    resizable: false
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

const ODROID_VENDOR_ID = '0525' // 장치 인식하기 위한 벤더 ID
const WATCH_INTERVAL = 200 // 장치 연결 여부 확인 주기
const ODROID_BAUD_RATE = 115200

let currentPort: string | null = null
let currentSerial: SerialPort | null = null

const pollSerial = async (win: BrowserWindow) => {
  const ports = await SerialPort.list()

  if (currentSerial) {
    if (!ports.find((port) => port.path === currentPort)) {
      currentSerial.close()
      currentSerial = null
      currentPort = null
    }

    return
  }

  const found = ports.find((port) => port.vendorId === ODROID_VENDOR_ID)

  if (!found) return

  currentPort = found.path
  currentSerial = new SerialPort({ path: currentPort, baudRate: ODROID_BAUD_RATE })

  currentSerial.on('data', (data) => {
    // TODO
  })

  currentSerial.on('close', () => {
    currentPort = null
    currentSerial = null
  })

  console.log(`Serial Port Connected: ${found.path}`)
  win.webContents.send('init', 'INIT')
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

  win.webContents.on('did-finish-load', () => {
    // Run interval when renderer is initialized
    setInterval(() => pollSerial(win), WATCH_INTERVAL)
  })
})

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    app.quit()
  }
})

ipcMain.on('system-state', (_, data: SystemState) => {
  console.log('SystemState has been modified:')
  console.log(data)

  if (!currentSerial || !currentSerial.isOpen) return

  const jsonString = JSON.stringify(data)

  currentSerial.flush()
  currentSerial.write(jsonString, (err) => {
    if (err) {
      console.log(`[${currentPort}]: ${err.message}`)
    }
  })
})
