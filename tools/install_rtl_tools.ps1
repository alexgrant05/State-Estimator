$ErrorActionPreference = "Stop"

$msysRoot = if ($env:MSYS2_ROOT) { $env:MSYS2_ROOT } else { "C:\msys64" }
$pacman = Join-Path $msysRoot "usr\bin\pacman.exe"

if (-not (Test-Path -LiteralPath $pacman -PathType Leaf)) {
    throw "MSYS2 was not found at $msysRoot. Install MSYS2, or set MSYS2_ROOT."
}

& $pacman -S --noconfirm --needed mingw-w64-ucrt-x86_64-verilator make
if ($LASTEXITCODE -ne 0) {
    throw "MSYS2 package installation failed with exit code $LASTEXITCODE"
}

$env:PATH = "$(Join-Path $msysRoot 'ucrt64\bin');$(Join-Path $msysRoot 'usr\bin');$env:PATH"
$perl = Join-Path $msysRoot "usr\bin\perl.exe"
$verilator = Join-Path $msysRoot "ucrt64\bin\verilator"

& $perl $verilator --version
& (Join-Path $msysRoot "ucrt64\bin\g++.exe") --version | Select-Object -First 1
& (Join-Path $msysRoot "usr\bin\make.exe") --version | Select-Object -First 1
python --version

Write-Host "RTL tools are installed. Run: python tools/run_rtl_tests.py"
