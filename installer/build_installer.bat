@echo off
rem Builds dist\SASDesigner with PyInstaller, then the Windows installer with Inno Setup.
rem Output: installer\Output\SASDesigner-<version>-setup.exe
setlocal
cd /d "%~dp0.."
for /f "tokens=2 delims==" %%v in ('findstr /b "__version__" version.py') do set VER=%%~v
set VER=%VER: =%
set VER=%VER:"=%
echo Version %VER%
"..\.venv\Scripts\pyinstaller.exe" --noconfirm --log-level WARN SASDesigner.spec || exit /b 1
set ISCC=%LOCALAPPDATA%\Programs\Inno Setup 6\ISCC.exe
if not exist "%ISCC%" set ISCC=%ProgramFiles(x86)%\Inno Setup 6\ISCC.exe
"%ISCC%" /Q /DMyAppVersion=%VER% installer\SASDesigner.iss || exit /b 1
echo Installer: installer\Output\SASDesigner-%VER%-setup.exe
