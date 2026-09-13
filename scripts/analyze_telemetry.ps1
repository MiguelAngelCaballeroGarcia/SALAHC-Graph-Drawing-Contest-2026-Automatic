param(
    [string]$LogFolder = "run-matrix-results/logs",
    [string]$LogGlob = "*.log",
    [int]$LowRemainingMsThreshold = 5,
    [string]$OutCsv = "run-matrix-results/telemetry-diagnostics.csv"
)

$ErrorActionPreference = "Stop"

if (-not (Test-Path $LogFolder)) {
    throw "Log folder not found: $LogFolder"
}

$logFiles = Get-ChildItem -Path $LogFolder -Filter $LogGlob | Sort-Object Name
if ($logFiles.Count -eq 0) {
    throw "No log files found in $LogFolder matching $LogGlob"
}

function Get-MetadataFromFileName {
    param([string]$BaseName)

    $graph = "Unknown"
    $budget = -1
    $seed = -1

    $mGraph = [regex]::Match($BaseName, '(Automatic-\d+)')
    if ($mGraph.Success) {
        $graph = $mGraph.Groups[1].Value
    }

    $mBudget = [regex]::Match($BaseName, '(?:^|[_-])(\d{4,7})(?:ms)?(?:[_-]|$)', [System.Text.RegularExpressions.RegexOptions]::IgnoreCase)
    if ($mBudget.Success) {
        $budget = [int]$mBudget.Groups[1].Value
    }

    $mSeed = [regex]::Match($BaseName, '(?:seed|s)(\d+)', [System.Text.RegularExpressions.RegexOptions]::IgnoreCase)
    if ($mSeed.Success) {
        $seed = [int64]$mSeed.Groups[1].Value
    }

    return [pscustomobject]@{
        Graph = $graph
        BudgetMs = $budget
        Seed = $seed
    }
}

function Parse-TelemetryLine {
    param([string]$Line)

    if ($Line -notmatch '\[PIPELINE\]\[telemetry\]') {
        return $null
    }

    $pairs = @{}
    foreach ($m in [regex]::Matches($Line, '([a-zA-Z0-9_]+)=([^\s]+)')) {
        $pairs[$m.Groups[1].Value] = $m.Groups[2].Value
    }

    if (-not $pairs.ContainsKey('phase') -or $pairs['phase'] -ne 'final_polish') {
        return $null
    }

    $asInt64 = {
        param($key)
        if ($pairs.ContainsKey($key)) {
            try { return [int64]$pairs[$key] } catch { return 0L }
        }
        return 0L
    }

    return [pscustomobject]@{
        InputIndex = & $asInt64 'input'
        RemainingBeforePolishMs = & $asInt64 'remaining_before_polish_ms'
        ReservedBudgetMs = & $asInt64 'reserved_budget_ms'
        PlannedBudgetMs = & $asInt64 'planned_budget_ms'
        ElapsedMs = & $asInt64 'elapsed_ms'
        ImprovedFlag = & $asInt64 'improved'
        PreK = & $asInt64 'pre_k'
        PostK = & $asInt64 'post_k'
        DeltaK = & $asInt64 'delta_k'
        PreCrossings = & $asInt64 'pre_crossings'
        PostCrossings = & $asInt64 'post_crossings'
        DeltaCrossings = & $asInt64 'delta_crossings'
        PreLp = & $asInt64 'pre_lp'
        PostLp = & $asInt64 'post_lp'
        DeltaLp = & $asInt64 'delta_lp'
    }
}

$rows = New-Object System.Collections.Generic.List[object]
foreach ($file in $logFiles) {
    $meta = Get-MetadataFromFileName -BaseName $file.BaseName
    $lines = Get-Content $file.FullName
    foreach ($line in $lines) {
        $parsed = Parse-TelemetryLine -Line $line
        if ($null -eq $parsed) {
            continue
        }

        $verdict = 'Neutral'
        if ($parsed.DeltaK -lt 0) {
            $verdict = 'InternalWin'
        } elseif ($parsed.DeltaK -gt 0) {
            $verdict = 'InternalRegression'
        } elseif ($parsed.RemainingBeforePolishMs -le $LowRemainingMsThreshold -and $parsed.ElapsedMs -eq 0) {
            $verdict = 'TimeStarved'
        }

        $rows.Add([pscustomobject]@{
            LogFile = $file.Name
            Graph = $meta.Graph
            BudgetMs = $meta.BudgetMs
            Seed = $meta.Seed
            InputIndex = $parsed.InputIndex
            RemainingBeforePolishMs = $parsed.RemainingBeforePolishMs
            ReservedBudgetMs = $parsed.ReservedBudgetMs
            PlannedBudgetMs = $parsed.PlannedBudgetMs
            PolishElapsedMs = $parsed.ElapsedMs
            ImprovedFlag = $parsed.ImprovedFlag
            PreK = $parsed.PreK
            PostK = $parsed.PostK
            DeltaK = $parsed.DeltaK
            PreCrossings = $parsed.PreCrossings
            PostCrossings = $parsed.PostCrossings
            DeltaCrossings = $parsed.DeltaCrossings
            PreLp = $parsed.PreLp
            PostLp = $parsed.PostLp
            DeltaLp = $parsed.DeltaLp
            Verdict = $verdict
        })
    }
}

if ($rows.Count -eq 0) {
    throw "No telemetry rows were parsed. Ensure logs contain [PIPELINE][telemetry] lines."
}

$rows | Export-Csv -Path $OutCsv -NoTypeInformation -Encoding UTF8
Write-Host "ParsedTelemetryRows=$($rows.Count)"
Write-Host "Saved telemetry diagnostics to $OutCsv"

Write-Host ""
Write-Host "TelemetrySummaryByGraphBudget:"
$rows |
    Group-Object Graph, BudgetMs |
    ForEach-Object {
        $g = $_.Group
        [pscustomobject]@{
            Graph = [string]$g[0].Graph
            BudgetMs = [int]$g[0].BudgetMs
            Samples = [int]$g.Count
            AvgRemainingMs = [math]::Round((($g | Measure-Object RemainingBeforePolishMs -Average).Average), 2)
            AvgPolishElapsedMs = [math]::Round((($g | Measure-Object PolishElapsedMs -Average).Average), 2)
            InternalWins = [int](($g | Where-Object { $_.DeltaK -lt 0 }).Count)
            InternalRegressions = [int](($g | Where-Object { $_.DeltaK -gt 0 }).Count)
            InternalNeutrals = [int](($g | Where-Object { $_.DeltaK -eq 0 }).Count)
            TimeStarved = [int](($g | Where-Object { $_.Verdict -eq 'TimeStarved' }).Count)
            MinDeltaK = [int64](($g | Measure-Object DeltaK -Minimum).Minimum)
            MaxDeltaK = [int64](($g | Measure-Object DeltaK -Maximum).Maximum)
        }
    } |
    Sort-Object BudgetMs, Graph |
    Format-Table -AutoSize
