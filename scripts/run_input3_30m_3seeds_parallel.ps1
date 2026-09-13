param(
    [string]$ExePath = "build-release/Release/SALAHC.exe",
    [string]$InputPath = "data/Automatic-4.json",
    [UInt64[]]$Seeds = @(1337, 4242, 7201, 9109, 11047, 13003, 15427, 18013, 20129),
    [int]$TimeLimitMs = 1800000,
    [string]$OutputRoot = "run-matrix-results"
)

$ErrorActionPreference = "Stop"

if ($Seeds.Count -ne 9) {
    throw "This script requires exactly 9 seeds. Provided: $($Seeds.Count)."
}

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$exeFull = Join-Path $repoRoot $ExePath
$inputFull = Join-Path $repoRoot $InputPath

if (-not (Test-Path $exeFull)) {
    throw "Executable not found: $exeFull"
}
if (-not (Test-Path $inputFull)) {
    throw "Input file not found: $inputFull"
}

$timestamp = Get-Date -Format "yyyyMMdd-HHmmss"
$outDir = Join-Path $repoRoot (Join-Path $OutputRoot ("input3-30m-9seeds-" + $timestamp))
New-Item -ItemType Directory -Path $outDir -Force | Out-Null

Write-Host "[run] exe=$exeFull"
Write-Host "[run] input=$inputFull"
Write-Host "[run] seeds=$($Seeds -join ',')"
Write-Host "[run] time_limit_ms=$TimeLimitMs"
Write-Host "[run] output_dir=$outDir"

$jobs = @()
foreach ($seed in $Seeds) {
    $logFile = Join-Path $outDir ("seed-" + $seed + ".log")
    $jobs += Start-Job -Name ("seed-" + $seed) -ScriptBlock {
        param($Exe, $InputFile, $Seed, $Ms, $Log, $WorkingDir)
        $ErrorActionPreference = "Continue"

        Set-Location -Path $WorkingDir

        $env:GDCONTESTAI_THREADS = "1"
        $env:GDCONTESTAI_TIME_LIMIT_MS = [string]$Ms
        $env:GDCONTESTAI_SEED = [string]$Seed

        try {
            $quotedExe = '"' + $Exe + '"'
            $quotedInput = '"' + $InputFile + '"'
            $quotedLog = '"' + $Log + '"'
            $cmdLine = "$quotedExe $quotedInput 1> $quotedLog 2>&1"
            cmd.exe /d /c $cmdLine | Out-Null
            if ($LASTEXITCODE -ne 0) {
                throw "Process failed with exit code $LASTEXITCODE"
            }
        } finally {
            Remove-Item Env:GDCONTESTAI_THREADS -ErrorAction SilentlyContinue
            Remove-Item Env:GDCONTESTAI_TIME_LIMIT_MS -ErrorAction SilentlyContinue
            Remove-Item Env:GDCONTESTAI_SEED -ErrorAction SilentlyContinue
        }
    } -ArgumentList $exeFull, $inputFull, $seed, $TimeLimitMs, $logFile, $repoRoot
}

Write-Host "[wait] waiting for 9 jobs to finish (~30 minutes)..."
Wait-Job -Job $jobs | Out-Null

$results = @()
foreach ($job in $jobs) {
    $seed = [UInt64]($job.Name -replace "seed-", "")
    $logFile = Join-Path $outDir ("seed-" + $seed + ".log")
    $state = $job.State

    try {
        Receive-Job -Job $job -Keep -ErrorAction Stop | Out-Null
    } catch {
        Write-Warning "Job $($job.Name) reported errors. See log file for details."
    }

    $summaryLine = ""
    if (Test-Path $logFile) {
        $summaryLine = (Select-String -Path $logFile -Pattern "\[PIPELINE\]\[summary\]" | Select-Object -Last 1).Line
    }

    $finalK = $null
    $tightTriggers = $null
    $maxNoImprove = $null
    $minFrontierSeen = $null
    $acceptedFrontierLe4 = $null

    if ($summaryLine) {
        $mK = [regex]::Match($summaryLine, "(?:^|\s)final_k=([0-9]+)")
        if ($mK.Success) { $finalK = [int]$mK.Groups[1].Value }

        $mT = [regex]::Match($summaryLine, "(?:^|\s)sa_tight_bottleneck_triggers=([0-9]+)")
        if ($mT.Success) { $tightTriggers = [Int64]$mT.Groups[1].Value }

        $mN = [regex]::Match($summaryLine, "(?:^|\s)sa_max_no_improve_streak=([0-9]+)")
        if ($mN.Success) { $maxNoImprove = [Int64]$mN.Groups[1].Value }

        $mF = [regex]::Match($summaryLine, "(?:^|\s)sa_min_frontier_seen_during_sa=([0-9]+)")
        if ($mF.Success) { $minFrontierSeen = [Int32]$mF.Groups[1].Value }

        $mA = [regex]::Match($summaryLine, "(?:^|\s)sa_accepted_moves_with_frontier_le4=([0-9]+)")
        if ($mA.Success) { $acceptedFrontierLe4 = [Int64]$mA.Groups[1].Value }
    }

    $results += [pscustomobject]@{
        Seed = $seed
        JobState = $state
        FinalK = $finalK
        SATightBottleneckTriggers = $tightTriggers
        SAMaxNoImproveStreak = $maxNoImprove
        SAMinFrontierSeenDuringSA = $minFrontierSeen
        SAAcceptedMovesWithFrontierLe4 = $acceptedFrontierLe4
        LogFile = $logFile
    }
}

$results = $results | Sort-Object Seed
$results | Format-Table -AutoSize

$csvPath = Join-Path $outDir "summary.csv"
$jsonPath = Join-Path $outDir "summary.json"
$results | Export-Csv -Path $csvPath -NoTypeInformation
$results | ConvertTo-Json -Depth 4 | Set-Content -Path $jsonPath -Encoding UTF8

Write-Host "[done] summary_csv=$csvPath"
Write-Host "[done] summary_json=$jsonPath"
Write-Host "[done] logs_dir=$outDir"

# Cleanup job objects
$jobs | Remove-Job -Force
