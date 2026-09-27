@echo off
for /f "tokens=5" %%P in ('netstat -ano ^| findstr ":8765"') do taskkill /PID %%P /F >nul 2>&1
 echo VORTEX preview server stopped if it was running.
