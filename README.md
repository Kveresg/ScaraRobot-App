# ScaraRobot-App

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)  
[![Version](https://img.shields.io/badge/version-1.0-blue)](https://github.com/<yourusername>/<yourapp>)

**ScaraRobot-App** is a Python-based application for Linux (WIP Windows).  
It includes an **interactive installer** that can:

- Check for Python 3 and install it if missing  
- Install the app as a **systemd service**  
- Enable/start the service automatically if desired  

Works on **Ubuntu, Debian, Fedora, Arch, openSUSE**, and most major Linux distributions.

---

## 📦 Project Structure

ScaraRobotApp/

├── install.sh # Interactive installer

├── uninstall.sh # Uninstaller

├── ScaraRobot # Built executable (PyInstaller or script)

├── README.md

└── LICENSE

---

## 🚀 Installation

### Option 1 – Clone via Git:

```bash
git clone https://github.com/Kveresg/ScaraRobot-App.git
cd <ScaraRobotApp>

chmod +x install.sh uninstall.sh
sudo ./install.sh
```
### Option 2 – Download ZIP from GitHub:
Go to the GitHub repository

Click Code → Download ZIP

Extract the archive and navigate to the folder:

```bash
cd <ScaraRobotApp>
chmod +x install.sh uninstall.sh
sudo ./install.sh
```

## ⚙️ Features
Interactive setup prompts:

- Install as a systemd service?

- Enable/start automatically?

- Install Python 3 if missing?

- Cross-distro compatibility

- Clean uninstallation with ./uninstall.sh

## 🧹 Uninstallation
```bash
cd <ScaraRobotApp>
sudo ./uninstall.sh
```
Removes:

- The executable from /usr/local/bin

- The systemd service file

- Stops and disables the service

## 🛠 Building the Executable (Optional)
If you want to rebuild from the Python source:

```bash
pip install pyinstaller
pyinstaller --onefile your_script.py
mv dist/your_script \<yourapp\>
```

## 🧩 Troubleshooting
Installer says Python 3 is missing
→ Allow it to install automatically, or install manually via your distro’s package manager.

Service fails to start
→ Run 
```bash
sudo journalctl -u ScaraRobotApp.service -f
```
to view logs.

Permission issues
→ Ensure you execute ``sudo ./install.sh`` for system-wide installation.

## 📄 License
This project is licensed under the MIT License.