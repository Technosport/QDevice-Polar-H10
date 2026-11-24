#define MyAppName "QDevice Polar H10 for QTM"
#define MyAppVersion "0.2.2"
#define MyAppPublisher "Technosport"
#define MyAppURL "https://github.com/Technosport/QDevice-polar-h10"

[Setup]
AppId={{8D6B2A1E-5B63-4D4B-9C9D-8F5D4E8A1020}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
AppPublisher={#MyAppPublisher}
AppPublisherURL={#MyAppURL}
AppSupportURL={#MyAppURL}
AppUpdatesURL={#MyAppURL}
DefaultDirName={autopf64}\Qualisys\Qualisys Track Manager\Input_Devices
DisableDirPage=no
DirExistsWarning=no
DisableProgramGroupPage=yes
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
PrivilegesRequired=admin
OutputDir=..\dist
OutputBaseFilename=QDevice-Polar-H10-Setup-v{#MyAppVersion}
Compression=lzma2
SolidCompression=yes
WizardStyle=modern
LicenseFile=..\LICENSE

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"
Name: "french"; MessagesFile: "compiler:Languages\French.isl"

[Files]
Source: "..\build\Release\qdevice_polar_h10.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\assets\qdevice_polar_h10.png"; DestDir: "{app}"; Flags: ignoreversion
