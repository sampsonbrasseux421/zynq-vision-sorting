# run_all.ps1 -- compile and run every testbench, writing logs to results/
#
# Requires Icarus Verilog 11+  (https://bleyer.org/icarus/)
# Usage:  pwsh -File run_all.ps1      from the sim/ directory
#
# Note: PowerShell variable names are case-insensitive, so the log directory and
# the compiled executable must not be called $OUT / $out.

$ErrorActionPreference = 'Continue'
Set-Location $PSScriptRoot

$Rtl  = '..\src\rtl'
$LogDir = 'results'
New-Item -ItemType Directory -Path $LogDir -Force | Out-Null

function Invoke-Tb {
    param([string]$Name, [string[]]$Src)

    $exe = "$Name.exe"
    & iverilog -g2012 -o $exe @Src 2>&1 |
        Out-File (Join-Path $LogDir "$Name.compile.txt") -Encoding utf8
    if ($LASTEXITCODE -ne 0) {
        Write-Host ('{0,-28}COMPILE FAILED' -f $Name)
        return
    }

    $log = & vvp $exe 2>&1
    $log | Out-File (Join-Path $LogDir "$Name.log") -Encoding utf8
    Remove-Item $exe -Force -ErrorAction SilentlyContinue

    $bad = ($log | Select-String -Pattern '\[FAIL\]|TIMEOUT').Count
    Write-Host ('{0,-28}{1}' -f $Name, $(if ($bad -gt 0) { 'FAIL' } else { 'PASS' }))
}

Invoke-Tb 'tb_zdt_frame_tx'        @('tb_zdt_frame_tx.v',        "$Rtl\zdt_frame_tx.v")
Invoke-Tb 'tb_zdt_tx_engine'       @('tb_zdt_tx_engine.v',       "$Rtl\zdt_tx_engine.v", "$Rtl\zdt_frame_tx.v")
Invoke-Tb 'tb_zdt_engine3'         @('tb_zdt_engine3.v',         "$Rtl\zdt_tx_engine.v", "$Rtl\zdt_frame_tx.v")
Invoke-Tb 'tb_zdt_sync'            @('tb_zdt_sync.v',            "$Rtl\zdt_tx_engine.v", "$Rtl\zdt_frame_tx.v")
Invoke-Tb 'tb_zdt_axil'            @('tb_zdt_axil.v',            "$Rtl\zdt_axil.v", "$Rtl\zdt_tx_engine.v", "$Rtl\zdt_frame_tx.v")
Invoke-Tb 'tb_color_blob_detector' @('tb_color_blob_detector.v', "$Rtl\color_blob_detector.v")
Invoke-Tb 'tb_roi_check'           @('tb_roi_check.v',           "$Rtl\color_blob_detector_roi.v", "$Rtl\color_blob_detector.v")

Write-Host ''
Write-Host "Logs written to $LogDir\"
