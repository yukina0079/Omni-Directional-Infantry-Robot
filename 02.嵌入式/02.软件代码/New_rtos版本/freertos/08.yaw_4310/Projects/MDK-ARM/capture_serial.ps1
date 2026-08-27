# Captures USART2 output to a log file.
#
# Reads RAW BYTES from the serial port rather than going through SerialPort's
# text decoder. The decoder maps anything outside the current encoding to '?'
# (0x3F), which silently destroys binary telemetry frames and makes it
# impossible to tell a real '?' from a mangled 0xFF.
#
# DTR/RTS are forced low: some USB-serial bridges assert them on open, and on a
# board that ties them to nRST/BOOT0 that would reset the target at exactly the
# wrong moment.

param(
    [string]$Port     = 'COM3',
    [int]   $Baud     = 115200,
    [string]$LogPath  = "$env:TEMP\yaw_selftest.log",
    [int]   $Seconds  = 30
)

if (Test-Path $LogPath) { Clear-Content $LogPath }

$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, 'None', 8, 'One'
$sp.DtrEnable    = $false
$sp.RtsEnable    = $false
$sp.ReadTimeout  = 200
$sp.ReadBufferSize = 65536
$sp.Open()

"listening on $Port @ $Baud -> $LogPath (${Seconds}s)"

$fs  = [System.IO.File]::Open($LogPath, 'Create', 'Write', 'Read')
$buf = New-Object byte[] 8192
$deadline = (Get-Date).AddSeconds($Seconds)
$total = 0

try {
    while ((Get-Date) -lt $deadline) {
        $avail = $sp.BytesToRead
        if ($avail -gt 0) {
            $n = $sp.BaseStream.Read($buf, 0, [Math]::Min($avail, $buf.Length))
            if ($n -gt 0) {
                $fs.Write($buf, 0, $n)
                $fs.Flush()
                $total += $n
            }
        } else {
            Start-Sleep -Milliseconds 20
        }
    }
}
finally {
    $fs.Close()
    $sp.Close()
}

"captured $total bytes"
