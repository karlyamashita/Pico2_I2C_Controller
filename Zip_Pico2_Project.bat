@echo off
setlocal
set "PROJECT=%~dp0"
for %%I in ("%PROJECT%.") do set "PROJECT_NAME=%%~nxI"
for %%I in ("%PROJECT%..") do set "PARENT=%%~fI"
set "DEST=%PARENT%\%PROJECT_NAME%_Source.zip"
echo Project: %PROJECT%
echo Output : %DEST%
echo.
powershell.exe -NoProfile -ExecutionPolicy Bypass -Command ^
  "$ErrorActionPreference='Stop'; Add-Type -AssemblyName System.IO.Compression; Add-Type -AssemblyName System.IO.Compression.FileSystem; $root=$env:PROJECT; $dest=$env:DEST; $projectName=$env:PROJECT_NAME; $excludedDirs=@('build','build-debug','build-release','.git','.vs','.vscode','bin','obj','Debug','Release','CMakeFiles','__pycache__'); $excludedExtensions=@('.uf2','.elf','.bin','.hex','.o','.obj','.a','.lib','.exe','.dll','.pdb','.map','.pyc','.zip'); if(Test-Path -LiteralPath $dest){Remove-Item -LiteralPath $dest -Force}; $zip=[System.IO.Compression.ZipFile]::Open($dest,[System.IO.Compression.ZipArchiveMode]::Create); try { Get-ChildItem -LiteralPath $root -File -Recurse -Force | ForEach-Object { $rel=$_.FullName.Substring($root.Length).TrimStart([char]92,[char]47); $segments=$rel.Split([char]92,[char]47); $skip=$false; foreach($segment in $segments){if($excludedDirs -contains $segment){$skip=$true;break}}; if(-not $skip -and $excludedExtensions -notcontains $_.Extension.ToLowerInvariant()){ $entry=($projectName+'/'+$rel.Replace([char]92,[char]47)); [void][System.IO.Compression.ZipFileExtensions]::CreateEntryFromFile($zip,$_.FullName,$entry,[System.IO.Compression.CompressionLevel]::Optimal) } } } finally { $zip.Dispose() }; Write-Host 'ZIP created successfully:' $dest"
if errorlevel 1 (
  echo.
  echo ERROR: ZIP creation failed.
  pause
  exit /b 1
)
echo.
pause
