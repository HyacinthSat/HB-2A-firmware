$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    $python = Join-Path $PSScriptRoot '.venv\Scripts\python.exe'
    if (!(Test-Path -LiteralPath $python)) {
        py -3 -m venv (Join-Path $PSScriptRoot '.venv')
        if ($LASTEXITCODE -ne 0) { throw 'Python 3 with tkinter is required.' }
    }
    & $python -c 'import serial'
    if ($LASTEXITCODE -ne 0) {
        & $python -m pip install -r (Join-Path $PSScriptRoot 'requirements.txt')
        if ($LASTEXITCODE -ne 0) { throw 'Could not install pyserial.' }
    }
    & $python (Join-Path $PSScriptRoot 'uplink_gui.py')
} finally {
    Pop-Location
}
