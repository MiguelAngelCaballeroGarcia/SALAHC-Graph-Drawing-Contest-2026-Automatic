$ErrorActionPreference = 'Continue'

$repoRoot = Split-Path -Parent $PSScriptRoot
$input = Join-Path $repoRoot "data/Automatic-1.json"
$baselineExe = Join-Path $repoRoot "bench-ab/GDContestAI.lbvh-baseline.exe"
$optimizedExe = Join-Path $repoRoot "bench-ab/GDContestAI.lbvh-optimized.exe"

if (-not (Test-Path $input)) { throw "Missing input file: $input" }
if (-not (Test-Path $baselineExe)) { throw "Missing baseline exe: $baselineExe" }
if (-not (Test-Path $optimizedExe)) { throw "Missing optimized exe: $optimizedExe" }

if (-not $env:GDCONTESTAI_THREADS) { $env:GDCONTESTAI_THREADS = "1" }
if (-not $env:GDCONTESTAI_TIME_LIMIT_MS) { $env:GDCONTESTAI_TIME_LIMIT_MS = "1000" }
if (-not $env:GDCONTESTAI_SEED) { $env:GDCONTESTAI_SEED = "20260627" }
$env:GDCONTESTAI_NO_TIME_LIMIT = "0"

function Run-One([string]$exe, [string]$label, [int]$runId) {
    Write-Host ("start " + $label + " run " + $runId)
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $prevErrorAction = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    & $exe $input 1>$null 2>$null
    $ErrorActionPreference = $prevErrorAction
    $sw.Stop()

    if ($LASTEXITCODE -ne 0) {
        throw ("Run failed for " + $label + " run " + $runId + " (exit=" + $LASTEXITCODE + ")")
    }

    $ms = $sw.Elapsed.TotalMilliseconds
    Write-Host ("done " + $label + " run " + $runId + " ms=" + [Math]::Round($ms, 3))
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

$warmupRunsEach = 1
$pairedRuns = 5

for ($i = 1; $i -le $warmupRunsEach; $i++) {
    [void](Run-One $baselineExe "baseline-warmup" $i)
    [void](Run-One $optimizedExe "optimized-warmup" $i)
}

$baselineTimes = New-Object System.Collections.Generic.List[Double]
$optimizedTimes = New-Object System.Collections.Generic.List[Double]

for ($i = 1; $i -le $pairedRuns; $i++) {
    $baselineTimes.Add((Run-One $baselineExe "baseline" $i))
    $optimizedTimes.Add((Run-One $optimizedExe "optimized" $i))
}

$baseline = Summ $baselineTimes
$optimized = Summ $optimizedTimes
$speedup = $baseline.mean_ms / $optimized.mean_ms
$improvementPct = (($baseline.mean_ms - $optimized.mean_ms) / $baseline.mean_ms) * 100.0

$pairedDelta = New-Object System.Collections.Generic.List[Double]
for ($i = 0; $i -lt $pairedRuns; $i++) {
    $pairedDelta.Add($baselineTimes[$i] - $optimizedTimes[$i])
}
$pairedMeanDelta = (Summ $pairedDelta).mean_ms

$result = [PSCustomObject]@{
    benchmark = "lbvh-only-ab"
    input = "data/Automatic-1.json"
    warmup_runs_each = $warmupRunsEach
    paired_runs = $pairedRuns
    env = [PSCustomObject]@{
        GDCONTESTAI_THREADS = $env:GDCONTESTAI_THREADS
        GDCONTESTAI_TIME_LIMIT_MS = $env:GDCONTESTAI_TIME_LIMIT_MS
        GDCONTESTAI_SEED = $env:GDCONTESTAI_SEED
        GDCONTESTAI_NO_TIME_LIMIT = $env:GDCONTESTAI_NO_TIME_LIMIT
    }
    baseline = $baseline
    optimized = $optimized
    paired_mean_delta_ms = [Math]::Round($pairedMeanDelta, 3)
    improvement_percent = [Math]::Round($improvementPct, 3)
    speedup_x = [Math]::Round($speedup, 4)
    raw_samples = [PSCustomObject]@{
        baseline_ms = $baselineTimes
        optimized_ms = $optimizedTimes
    }
}

$out = Join-Path $repoRoot "bench-ab/ab_benchmark_lbvh_input0.json"
$result | ConvertTo-Json -Depth 6 | Set-Content -Path $out

Write-Output "RESULT_JSON"
$result | ConvertTo-Json -Depth 6
