; Inno Setup script for SAS & Autopilot Designer.
; Build:  installer\build_installer.bat   (runs PyInstaller, then ISCC with the version from version.py)
; Per-user install by default (no administrator rights); "install for all users" is offered in the wizard.

#ifndef MyAppVersion
  #define MyAppVersion "1.0.1"
#endif
#define MyAppName "SAS & Autopilot Designer"
#define MyAppExe "SASDesigner.exe"

[Setup]
AppId={{A8BCF72D-4B91-48EE-935A-EC142EB2FD4B}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
AppVerName={#MyAppName} {#MyAppVersion}
AppPublisher={#MyAppName}
DefaultDirName={autopf}\SAS Autopilot Designer
DefaultGroupName={#MyAppName}
DisableProgramGroupPage=yes
PrivilegesRequired=lowest
PrivilegesRequiredOverridesAllowed=dialog
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
OutputDir=Output
OutputBaseFilename=SASDesigner-{#MyAppVersion}-setup
SetupIconFile=..\data\app.ico
UninstallDisplayIcon={app}\{#MyAppExe}
UninstallDisplayName={#MyAppName} {#MyAppVersion}
Compression=lzma2/max
SolidCompression=yes
WizardStyle=modern
ChangesAssociations=yes
VersionInfoVersion={#MyAppVersion}
VersionInfoProductName={#MyAppName}

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "Create a &desktop shortcut"; GroupDescription: "Additional shortcuts:"
Name: "fileassoc"; Description: "Open .sasproj project files with {#MyAppName}"; GroupDescription: "File association:"

[Files]
Source: "..\dist\SASDesigner\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs
Source: "..\README.md"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\LICENSE"; DestDir: "{app}"; DestName: "LICENSE.txt"; Flags: ignoreversion

[Icons]
Name: "{group}\{#MyAppName}"; Filename: "{app}\{#MyAppExe}"
Name: "{group}\User guide (README)"; Filename: "{app}\README.md"
Name: "{group}\Uninstall {#MyAppName}"; Filename: "{uninstallexe}"
Name: "{autodesktop}\{#MyAppName}"; Filename: "{app}\{#MyAppExe}"; Tasks: desktopicon

[Registry]
Root: HKA; Subkey: "Software\Classes\.sasproj"; ValueType: string; ValueName: ""; ValueData: "SASDesigner.Project"; Flags: uninsdeletevalue uninsdeletekeyifempty; Tasks: fileassoc
Root: HKA; Subkey: "Software\Classes\SASDesigner.Project"; ValueType: string; ValueName: ""; ValueData: "SAS & Autopilot Designer project"; Flags: uninsdeletekey; Tasks: fileassoc
Root: HKA; Subkey: "Software\Classes\SASDesigner.Project\DefaultIcon"; ValueType: string; ValueName: ""; ValueData: "{app}\{#MyAppExe},0"; Tasks: fileassoc
Root: HKA; Subkey: "Software\Classes\SASDesigner.Project\shell\open\command"; ValueType: string; ValueName: ""; ValueData: """{app}\{#MyAppExe}"" ""%1"""; Tasks: fileassoc

[Run]
Filename: "{app}\{#MyAppExe}"; Description: "Launch {#MyAppName}"; Flags: nowait postinstall skipifsilent

[UninstallDelete]
Type: filesandordirs; Name: "{app}\_internal"
