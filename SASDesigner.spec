# PyInstaller spec:  ..\.venv\Scripts\pyinstaller.exe --noconfirm SASDesigner.spec
import re

from PyInstaller.utils.hooks import collect_dynamic_libs, collect_submodules
from PyInstaller.utils.win32.versioninfo import (FixedFileInfo, StringFileInfo, StringStruct, StringTable,
                                                 VarFileInfo, VarStruct, VSVersionInfo)

VERSION = re.search(r'__version__ = "([^"]+)"', open("version.py", encoding="utf-8").read()).group(1)
vt = tuple(int(x) for x in VERSION.split(".")) + (0,) * (4 - len(VERSION.split(".")))

hidden = collect_submodules("control") + collect_submodules("slycot") + ["scipy.io.matlab"]
binaries = collect_dynamic_libs("slycot")

excludes = [
    "PySide6.QtWebEngineCore", "PySide6.QtWebEngineWidgets", "PySide6.QtWebEngineQuick",
    "PySide6.QtQuick", "PySide6.QtQml", "PySide6.Qt3DCore", "PySide6.QtMultimedia",
    "PySide6.QtCharts", "PySide6.QtDataVisualization", "PySide6.QtPdf", "PySide6.QtBluetooth",
    "PySide6.QtLocation", "PySide6.QtPositioning", "PySide6.QtSensors", "PySide6.QtSerialPort",
    "PySide6.QtSql", "PySide6.QtTest", "PySide6.QtNetworkAuth", "PySide6.QtRemoteObjects",
    "tkinter", "IPython", "pytest",
]

version_info = VSVersionInfo(
    ffi=FixedFileInfo(filevers=vt, prodvers=vt),
    kids=[StringFileInfo([StringTable("040904B0", [
        StringStruct("CompanyName", "SAS & Autopilot Designer"),
        StringStruct("FileDescription", "SAS & Autopilot Designer"),
        StringStruct("FileVersion", VERSION),
        StringStruct("ProductName", "SAS & Autopilot Designer"),
        StringStruct("ProductVersion", VERSION),
        StringStruct("OriginalFilename", "SASDesigner.exe")])]),
        VarFileInfo([VarStruct("Translation", [0x0409, 1200])])])

a = Analysis(
    ["main.py"],
    pathex=["."],
    binaries=binaries,
    datas=[("data/app.ico", "data")],
    hiddenimports=hidden,
    excludes=excludes,
    noarchive=False,
)
pyz = PYZ(a.pure)
exe = EXE(pyz, a.scripts, [], exclude_binaries=True, name="SASDesigner", console=False,
          upx=False, icon="data/app.ico", version=version_info)
coll = COLLECT(exe, a.binaries, a.datas, upx=False, name="SASDesigner")
