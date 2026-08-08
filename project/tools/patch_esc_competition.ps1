param(
    [string]$EscRoot = "E:\CYT2BL3_Brushless_Driver_4IN1_Projec-6.11t-master\CYT2BL3_Brushless_Driver_4IN1_Project-master\CYT2BL3_Brushless_Driver_4IN1_Project"
)

$ErrorActionPreference = "Stop"
$latin1 = [System.Text.Encoding]::GetEncoding(28591)

function Update-Latin1File {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Replacement,
        [int]$ExpectedCount
    )

    $bytes = [System.IO.File]::ReadAllBytes($Path)
    $text = $latin1.GetString($bytes)
    $matches = [System.Text.RegularExpressions.Regex]::Matches($text, $Pattern)
    if ($matches.Count -ne $ExpectedCount) {
        throw "Unexpected match count in ${Path}: expected ${ExpectedCount}, got $($matches.Count)"
    }

    $updated = [System.Text.RegularExpressions.Regex]::Replace($text, $Pattern, $Replacement)
    [System.IO.File]::WriteAllBytes($Path, $latin1.GetBytes($updated))
}

$uartPath = Join-Path $EscRoot "project\code\motor_driver_uart_control.c"
# Flight control only uses 0x01 duty frames. Make every UART SET_ZERO parser
# path a no-op while leaving the internal enum and boot-time self-test intact.
Update-Latin1File `
    -Path $uartPath `
    -Pattern '(communicat_value->immediate_command\s*=\s*)SET_ZERO;' `
    -Replacement '${1}NULL_CMD;' `
    -ExpectedCount 4

Write-Output "ESC competition patch applied: runtime UART SET_ZERO disabled."
