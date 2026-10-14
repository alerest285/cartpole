@echo off
cd /d "%~dp0"
.venv\Scripts\python -m cartpole doctor --hardware
if errorlevel 1 goto failed
echo At the rig only: stop all QUARC models; centre the cart and hang the pole.
echo The console opens disarmed. Read docs\lab-runbook.md before pressing Arm.
pause
.venv\Scripts\python -m cartpole serve --backend quanser
pause
exit /b 0
:failed
echo Install Quanser's API into .venv before starting the hardware console.
pause
exit /b 1
