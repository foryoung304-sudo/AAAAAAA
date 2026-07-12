Add-Type -AssemblyName System.Windows.Forms.DataVisualization
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path "$PSScriptRoot\\..").Path
$out = Join-Path $PSScriptRoot 'figures_png'
New-Item -ItemType Directory -Force -Path $out | Out-Null

function New-LineChart($file, $title, $yTitle, $seriesSpec) {
    $chart = New-Object System.Windows.Forms.DataVisualization.Charting.Chart
    $chart.Width = 1200; $chart.Height = 700; $chart.BackColor = [System.Drawing.Color]::White
    $area = New-Object System.Windows.Forms.DataVisualization.Charting.ChartArea 'Main'
    $area.BackColor = [System.Drawing.Color]::White
    $area.AxisX.Title = 'Time (s)'; $area.AxisY.Title = $yTitle
    $area.AxisX.MajorGrid.LineColor = [System.Drawing.Color]::FromArgb(225,230,236)
    $area.AxisY.MajorGrid.LineColor = [System.Drawing.Color]::FromArgb(225,230,236)
    $area.AxisX.LabelStyle.Format = '0.0'; $area.AxisY.LabelStyle.Format = '0.0'
    $chart.ChartAreas.Add($area)
    $legend = New-Object System.Windows.Forms.DataVisualization.Charting.Legend 'Legend'
    $legend.Docking = 'Top'; $legend.Alignment = 'Center'; $chart.Legends.Add($legend)
    foreach($spec in $seriesSpec) {
        $s = New-Object System.Windows.Forms.DataVisualization.Charting.Series $spec.Name
        $s.ChartType = [System.Windows.Forms.DataVisualization.Charting.SeriesChartType]::Line
        $s.BorderWidth = $spec.Width; $s.Color = $spec.Color
        $s.ChartArea = 'Main'; $s.Legend = 'Legend'
        foreach($p in $spec.Data){ [void]$s.Points.AddXY($p[0],$p[1]) }
        $chart.Series.Add($s)
    }
    $chart.Titles.Add($title).Font = New-Object System.Drawing.Font('Arial',16,[System.Drawing.FontStyle]::Bold)
    $chart.SaveImage((Join-Path $out $file), [System.Windows.Forms.DataVisualization.Charting.ChartImageFormat]::Png)
    $chart.Dispose()
}

$lines = Get-Content (Join-Path $root 'project\tools\height_log.txt')
$h = ($lines | Where-Object { $_ -like 'seq,*' } | Select-Object -First 1).Split(',')
$headerIndex = [array]::IndexOf($lines, ($lines | Where-Object { $_ -like 'seq,*' } | Select-Object -First 1))
$body = $lines[($headerIndex + 1)..($lines.Length - 1)] | Where-Object { $_ -match '^\d+,' }
$rows = $body | ConvertFrom-Csv -Header $h
$t0 = [double]$rows[0].time_us
function Pts($field) { return @($rows | ForEach-Object { $x = (([double]$_.time_us - $t0) / 1e6); $y = [double]($_.$field); ,@($x, $y) }) }
New-LineChart 'fig_height_estimation.png' 'Altitude-estimation log' 'Height (cm)' @(
 @{Name='ToF measurement';Color=[System.Drawing.Color]::SlateGray;Width=1;Data=(Pts 'tof_cm')},
 @{Name='EKF height';Color=[System.Drawing.Color]::RoyalBlue;Width=3;Data=(Pts 'ekf_z_cm')},
 @{Name='Control feedback';Color=[System.Drawing.Color]::DarkOrange;Width=2;Data=(Pts 'final_height_cm')},
 @{Name='Profile reference';Color=[System.Drawing.Color]::DimGray;Width=2;Data=(Pts 'profile_height_cm')})
New-LineChart 'fig_vertical_state.png' 'Vertical-state log' 'Vertical velocity (cm/s)' @(
 @{Name='EKF vertical velocity';Color=[System.Drawing.Color]::RoyalBlue;Width=3;Data=(Pts 'ekf_vz_cm_s')},
 @{Name='Profile vertical velocity';Color=[System.Drawing.Color]::DarkOrange;Width=2;Data=(Pts 'profile_vz_cm_s')})

$debug = Get-Content (Join-Path $root 'project\tools\debug_log_2.txt') -Raw
$m = [regex]::Matches($debug, 'roll=([-+0-9.]+),\s*pitch=([-+0-9.]+),yaw=([-+0-9.]+)')
$roll = @(); $pitch = @(); for($i=0;$i -lt $m.Count;$i++){ $roll += ,@($i,[double]$m[$i].Groups[1].Value); $pitch += ,@($i,[double]$m[$i].Groups[2].Value) }
New-LineChart 'fig_attitude_debug.png' 'Attitude samples from debug log' 'Angle (deg)' @(
 @{Name='Roll';Color=[System.Drawing.Color]::RoyalBlue;Width=3;Data=$roll},
 @{Name='Pitch';Color=[System.Drawing.Color]::DarkOrange;Width=3;Data=$pitch})

$c = New-Object System.Windows.Forms.DataVisualization.Charting.Chart; $c.Width=1000; $c.Height=620
$a = New-Object System.Windows.Forms.DataVisualization.Charting.ChartArea 'Main'; $a.AxisY.Title='Residual / error (cm)'; $a.AxisX.MajorGrid.Enabled=$false; $c.ChartAreas.Add($a)
$s = New-Object System.Windows.Forms.DataVisualization.Charting.Series 'Measured result'; $s.ChartType='Column'; $s.Color=[System.Drawing.Color]::RoyalBlue
[void]$s.Points.AddXY('Height error',3.0); [void]$s.Points.AddXY('Body X residual',1.43); [void]$s.Points.AddXY('Body Y residual',3.14); [void]$s.Points.AddXY('Earth X residual',1.51); [void]$s.Points.AddXY('Earth Y residual',9.42); $c.Series.Add($s); $c.Titles.Add('Measured control and closure results').Font=New-Object System.Drawing.Font('Arial',16,[System.Drawing.FontStyle]::Bold); $c.SaveImage((Join-Path $out 'fig_test_summary.png'),'Png'); $c.Dispose()
