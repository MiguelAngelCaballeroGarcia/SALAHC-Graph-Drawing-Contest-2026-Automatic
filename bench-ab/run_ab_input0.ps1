$ErrorActionPreference = 'Continue'

$repoRoot = Split-Path -Parent $PSScriptRoot
$input = Join-Path $repoRoot "data/Automatic-1.json"
$optExe = Join-Path $repoRoot "bench-ab/GDContestAI.optimized.exe"
$baseExe = Join-Path $repoRoot "bench-ab/GDContestAI.baseline.exe"

if (-not $env:GDCONTESTAI_THREADS) { $env:GDCONTESTAI_THREADS = "1" }
if (-not $env:GDCONTESTAI_TIME_LIMIT_MS) { $env:GDCONTESTAI_TIME_LIMIT_MS = "1000" }
if (-not $env:GDCONTESTAI_SEED) { $env:GDCONTESTAI_SEED = "20260627" }
Remove-Item Env:GDCONTESTAI_NO_TIME_LIMIT -ErrorAction SilentlyContinue

function Run-One([string]$exe, [string]$label, [int]$i) {
    [Console]::WriteLine("start " + $label + " run " + $i)
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    & $exe $input 1>$null 2>$null
    $sw.Stop()
    if ($LASTEXITCODE -ne 0) {
        throw ("Run failed: " + $label + " #" + $i + " exit=" + $LASTEXITCODE)
    }

    $ms = $sw.Elapsed.TotalMilliseconds
    [Console]::WriteLine("done " + $label + " run " + $i + " ms=" + [Math]::Round($ms, 3))
    return $ms
}

function Summ([System.Collections.Generic.List[Double]]$times) {
    $arr = $times.ToArray()
    [Array]::Sort($arr)

    $sum = 0.0
    foreach ($t in $arr) { $sum += $t }
    $mean = $sum / $arr.Length

    $median = if ($arr.Length % 2 -eq 1) {
        $arr[[int]($arr.Length / 2)]
    } else {
        ($arr[$arr.Length / 2 - 1] + $arr[$arr.Length / 2]) / 2.0
    }

    $var = 0.0
    foreach ($t in $arr) {
        $d = $t - $mean
        $var += $d * $d
    }

    $std = [Math]::Sqrt($var / $arr.Length)

    return [PSCustomObject]@{
        runs = $arr.Length
        min_ms = [Math]::Round($arr[0], 3)
        median_ms = [Math]::Round($median, 3)
        max_ms = [Math]::Round($arr[$arr.Length - 1], 3)
        mean_ms = [Math]::Round($mean, 3)
        std_ms = [Math]::Round($std, 3)
    }
}

$warmup = 1
for ($i = 1; $i -le $warmup; $i++) {
    [void](Run-One $baseExe "baseline-warmup" $i)
    [void](Run-One $optExe "optimized-warmup" $i)
}

$runs = 8
$baseTimes = New-Object System.Collections.Generic.List[Double]
$optTimes = New-Object System.Collections.Generic.List[Double]

for ($i = 1; $i -le $runs; $i++) {
    $baseTimes.Add((Run-One $baseExe "baseline" $i))
    $optTimes.Add((Run-One $optExe "optimized" $i))
}

$base = Summ $baseTimes
$opt = Summ $optTimes
$speedup = $base.mean_ms / $opt.mean_ms
$impr = (($base.mean_ms - $opt.mean_ms) / $base.mean_ms) * 100.0

$paired = New-Object System.Collections.Generic.List[Double]
for ($i = 0; $i -lt $runs; $i++) {
    $paired.Add($baseTimes[$i] - $optTimes[$i])
}
$pairedMean = (Summ $paired).mean_ms

$result = [PSCustomObject]@{
    input = "data/Automatic-1.json"
    warmup_runs_each = $warmup
    paired_runs = $runs
    env = [PSCustomObject]@{
        GDCONTESTAI_THREADS = $env:GDCONTESTAI_THREADS
        GDCONTESTAI_TIME_LIMIT_MS = $env:GDCONTESTAI_TIME_LIMIT_MS
        GDCONTESTAI_SEED = $env:GDCONTESTAI_SEED
    }
    baseline = $base
    optimized = $opt
    paired_mean_delta_ms = [Math]::Round($pairedMean, 3)
    improvement_percent = [Math]::Round($impr, 3)
    speedup_x = [Math]::Round($speedup, 4)
    raw_samples = [PSCustomObject]@{
        baseline_ms = $baseTimes
        optimized_ms = $optTimes
    }
}

$out = Join-Path $repoRoot "bench-ab/ab_benchmark_input0.json"
$result | ConvertTo-Json -Depth 6 | Set-Content -Path $out

Write-Output "RESULT_JSON"
$result | ConvertTo-Json -Depth 6
