@echo off
setlocal
set "ROOT=%~dp0"
for %%I in ("%ROOT%.") do set "NAME=%%~nxI"
for %%I in ("%ROOT%..") do set "PARENT=%%~fI"
set "OUT=%PARENT%\%NAME%_Source.zip"
echo Project: %ROOT%
echo Output : %OUT%
powershell -NoProfile -ExecutionPolicy Bypass -Command "$ErrorActionPreference='Stop'; $root=(Resolve-Path -LiteralPath '%ROOT%').Path.TrimEnd([char]92); $parent=Split-Path -Parent $root; $name=Split-Path -Leaf $root; $out=Join-Path $parent ($name+'_Source.zip'); Add-Type -AssemblyName System.IO.Compression.FileSystem; if(Test-Path -LiteralPath $out){Remove-Item -LiteralPath $out -Force}; $zip=[System.IO.Compression.ZipFile]::Open($out,'Create'); try {Get-ChildItem -LiteralPath $root -File -Recurse | Where-Object { $rel=$_.FullName.Substring($root.Length).TrimStart([char]92); $parts=$rel.Split([char]92); -not ($parts | Where-Object {$_ -in @('build','.git','.vs','.vscode','out','bin','obj')}) -and $_.Extension -notin @('.uf2','.elf','.bin','.hex','.map','.o','.obj','.a','.exe','.dll','.zip') } | ForEach-Object { $rel=$_.FullName.Substring($root.Length).TrimStart([char]92).Replace([char]92,[char]47); [System.IO.Compression.ZipFileExtensions]::CreateEntryFromFile($zip,$_.FullName,($name+'/'+$rel)) | Out-Null } } finally {$zip.Dispose()}; Write-Host ('Created: '+$out)"
if errorlevel 1 (echo ZIP failed.) else (echo ZIP complete.)
pause
