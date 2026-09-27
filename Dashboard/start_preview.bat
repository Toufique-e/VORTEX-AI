@echo off
setlocal
cd /d "%~dp0"
set "PORT=8765"
set "URL=http://127.0.0.1:%PORT%/"

echo Starting VORTEX local dashboard preview...
where py >nul 2>&1
if %errorlevel%==0 (
    start "VORTEX Preview Server" /B py -3 preview_server.py
) else (
    where python >nul 2>&1
    if errorlevel 1 (
        echo Python 3 was not found. Install Python 3 and run this file again.
        pause
        exit /b 1
    )
    start "VORTEX Preview Server" /B python preview_server.py
)

echo Waiting for preview server on %URL%
for /l %%I in (1,1,30) do (
    powershell -NoProfile -Command "try { $r=Invoke-WebRequest -UseBasicParsing -TimeoutSec 1 '%URL%'; if($r.StatusCode -eq 200){exit 0}else{exit 1} } catch { exit 1 }" >nul 2>&1
    if not errorlevel 1 goto READY
    timeout /t 1 /nobreak >nul
)

echo Preview server did not become ready within 30 seconds.
echo Dashboard URL: %URL%
echo Settings URL:  http://127.0.0.1:%PORT%/settings
pause
exit /b 1

:READY
start "" "%URL%"
echo Dashboard: %URL%
echo Settings:  http://127.0.0.1:%PORT%/settings
endlocal
