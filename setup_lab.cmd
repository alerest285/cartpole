@echo off
cd /d "%~dp0"
python -m venv .venv
if errorlevel 1 goto failed
.venv\Scripts\python -m pip install -r requirements.txt
if errorlevel 1 goto failed
.venv\Scripts\python -m cartpole doctor
if errorlevel 1 goto failed
echo Setup complete. Run start_sim.cmd to rehearse before connecting the rig.
pause
exit /b 0
:failed
echo Setup failed. Review the error above; no hardware was opened.
pause
exit /b 1
