@echo off
setlocal
cd /d "%~dp0"
if defined HB2A_PYTHON goto run
if exist "%USERPROFILE%\radioconda\python.exe" set "HB2A_PYTHON=%USERPROFILE%\radioconda\python.exe"
if not defined HB2A_PYTHON if defined CONDA_PREFIX if exist "%CONDA_PREFIX%\python.exe" set "HB2A_PYTHON=%CONDA_PREFIX%\python.exe"
if not defined HB2A_PYTHON (
  echo Activate the conda environment from environment.yml or set HB2A_PYTHON.
  pause
  exit /b 1
)
:run
echo Using Python: %HB2A_PYTHON%
"%HB2A_PYTHON%" build_native.py
if errorlevel 1 goto failed
if not exist build\hb2a_wire.dll if not exist build\hb2a_dsp.dll (
  echo Native DLLs were not produced.
  goto failed
)
"%HB2A_PYTHON%" station.py %*
if errorlevel 1 goto failed
exit /b 0
:failed
pause
exit /b 1

