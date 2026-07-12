Add-Type -AssemblyName System.Drawing
$root = (Resolve-Path "$PSScriptRoot\..").Path
$out = Join-Path $PSScriptRoot 'code_png'; New-Item -ItemType Directory -Force -Path $out | Out-Null
function Make-CodeImage($source,$start,$count,$name) {
  $lines = Get-Content $source -Encoding UTF8 | Select-Object -Skip ($start-1) -First $count
  $font = New-Object System.Drawing.Font('Consolas',9)
  $bmp = New-Object System.Drawing.Bitmap(1500,([Math]::Max(220,($lines.Count+3)*22)))
  $g = [System.Drawing.Graphics]::FromImage($bmp); $g.Clear([System.Drawing.Color]::FromArgb(30,34,42))
  $brush = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(222,226,230))
  $num = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(135,145,160))
  for($i=0;$i -lt $lines.Count;$i++){ $y=20+$i*22; $g.DrawString(('{0,4}' -f ($start+$i)), $font, $num, 20, $y); $g.DrawString($lines[$i],$font,$brush,100,$y) }
  $bmp.Save((Join-Path $out $name),[System.Drawing.Imaging.ImageFormat]::Png); $g.Dispose(); $bmp.Dispose(); $font.Dispose()
}
Make-CodeImage (Join-Path $root 'project\code\loc_ctrl.c') 230 62 'code_location_cascade.png'
Make-CodeImage (Join-Path $root 'project\code\fly_ctrl.c') 1 70 'code_motor_mixing.png'
