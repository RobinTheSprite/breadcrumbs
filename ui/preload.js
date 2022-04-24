const { contextBridge, ipcRenderer } = require('electron')

contextBridge.exposeInMainWorld("api", {
  generateRoute: formData => ipcRenderer.send('generateRoute', formData)
})
