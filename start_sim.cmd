@echo off
cd /d "%~dp0"
.venv\Scripts\python -m cartpole serve --backend sim --demo
pause
