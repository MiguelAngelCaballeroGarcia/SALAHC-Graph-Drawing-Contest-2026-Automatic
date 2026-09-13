param(
    [string]$SeedFile = "run-matrix-results/canonical-seeds-7-20260620-155939.txt",
    [string]$BinaryPath = "build-release/Release/SALAHC.exe",
    [string]$GraphPattern = "data/Automatic-*.json",
    [string[]]$Graphs = @("Automatic-8", "Automatic-9"),
    [int[]]$BudgetsMs = @(20000, 60000),
    [UInt64[]]$Seeds = @(),
    [int]$ThreadsPerRun = 1,
    [string]$OutputFile = "run-matrix-results/in-process-paired-metrics.csv"
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $repoRoot

$exe = (Resolve-Path $BinaryPath -ErrorAction Stop).Path

$files = Get-ChildItem -Path $GraphPattern | Sort-Object Name
if ($Graphs.Count -gt 0) {
    $wanted = @{}
    foreach ($g in $Graphs) { $wanted[$g] = $true }
    $files = @($files | Where-Object { $wanted.ContainsKey([System.IO.Path]::GetFileNameWithoutExtension($_.Name)) })
}
if ($files.Count -eq 0) {
    throw "No graph files selected by GraphPattern/Graphs filter."
}

if ($Seeds.Count -eq 0) {
    if (-not (Test-Path $SeedFile)) {
        throw "Seed file not found: $SeedFile"
    }
    $seedLines = Get-Content $SeedFile
    foreach ($line in $seedLines) {
        $trimmed = $line.Trim()
        if ($trimmed -match '^\d+$') {
            $Seeds += [UInt64]$trimmed
        }
    }
    if ($Seeds.Count -eq 0) {
        throw "No numeric seeds found in $SeedFile"
    }
}

function Get-IntFromText {
    param(
        [string]$Text,
        [string]$Pattern
    )

    $m = [regex]::Match($Text, $Pattern)
    if ($m.Success) {
        return [int64]$m.Groups[1].Value
    }
    return -1
}

function Get-BoolFromText {
    param(
        [string]$Text,
        [string]$Pattern
    )

    $m = [regex]::Match($Text, $Pattern)
    if ($m.Success) {
        return ($m.Groups[1].Value -eq "1")
    }
    return $false
}

function Run-One {
    param(
        [string]$Exe,
        [string]$InputPath,
        [int]$BudgetMs,
        [UInt64]$Seed,
        [int]$Threads,
        [bool]$DisablePolish
    )

    $env:GDCONTESTAI_POLISH_TELEMETRY = "1"
    $env:GDCONTESTAI_TIME_LIMIT_MS = [string]$BudgetMs
    $env:GDCONTESTAI_SEED = [string]$Seed
    $env:GDCONTESTAI_THREADS = [string]$Threads
    $env:GDCONTESTAI_DISABLE_FINAL_POLISH = if ($DisablePolish) { "1" } else { "0" }

    $tmp = Join-Path $env:TEMP ([guid]::NewGuid().ToString() + ".log")
    $previousErrorActionPreference = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    & $Exe $InputPath *> $tmp
    $ErrorActionPreference = $previousErrorActionPreference
    $output = if (Test-Path $tmp) { Get-Content $tmp -Raw } else { "" }
    Remove-Item $tmp -ErrorAction SilentlyContinue
    $finalK = Get-IntFromText -Text $output -Pattern 'final_k=(\d+)'
    $remaining = Get-IntFromText -Text $output -Pattern 'remaining_before_polish_ms=(-?\d+)'
    $enabled = Get-BoolFromText -Text $output -Pattern 'enabled=(\d+)'
    $reservedBudgetMs = Get-IntFromText -Text $output -Pattern 'reserved_budget_ms=(-?\d+)'
    $plannedBudgetMs = Get-IntFromText -Text $output -Pattern 'planned_budget_ms=(-?\d+)'
    $elapsedPolishMs = Get-IntFromText -Text $output -Pattern 'elapsed_ms=(-?\d+)'

    return [pscustomobject]@{
        Output = $output
        FinalK = $finalK
        RemainingBeforePolishMs = $remaining
        Enabled = $enabled
        ReservedBudgetMs = $reservedBudgetMs
        PlannedBudgetMs = $plannedBudgetMs
        ElapsedPolishMs = $elapsedPolishMs
    }
}

Write-Host "=== LAUNCHING IN-PROCESS BACK-TO-BACK PAIRED SWEEP ==="
Write-Host "graphs=$($files.Count) seeds=$($Seeds.Count) budgets=$($BudgetsMs -join ',')"

$rows = New-Object System.Collections.Generic.List[object]

foreach ($budget in $BudgetsMs) {
    foreach ($file in $files) {
        $graphName = [System.IO.Path]::GetFileNameWithoutExtension($file.Name)
        foreach ($seed in $Seeds) {
            Write-Host "slice graph=$graphName budget=$budget seed=$seed"

            $baseline = Run-One -Exe $exe -InputPath $file.FullName -BudgetMs $budget -Seed $seed -Threads $ThreadsPerRun -DisablePolish:$false
            $ablated = Run-One -Exe $exe -InputPath $file.FullName -BudgetMs $budget -Seed $seed -Threads $ThreadsPerRun -DisablePolish:$true

            $delta = [int64]($ablated.FinalK - $baseline.FinalK)
            $polishEligible = ($baseline.Enabled -and $baseline.PlannedBudgetMs -gt 0)
            $polishExecuted = ($polishEligible -and $baseline.ElapsedPolishMs -gt 0)
            $starved = ($polishEligible -and $baseline.RemainingBeforePolishMs -le 0)

            $rows.Add([pscustomobject]@{
                Graph = $graphName
                BudgetMs = [int]$budget
                Seed = [UInt64]$seed
                BaselineK = [int64]$baseline.FinalK
                AblatedK = [int64]$ablated.FinalK
                PairedDeltaK = $delta
                PolishEligible = $polishEligible
                PolishExecuted = $polishExecuted
                RemainingBeforePolishMs = [int64]$baseline.RemainingBeforePolishMs
                PlannedPolishBudgetMs = [int64]$baseline.PlannedBudgetMs
                ElapsedPolishMs = [int64]$baseline.ElapsedPolishMs
                Starved = $starved
            })

            Write-Host ("  -> baseline_k={0} ablated_k={1} delta={2} eligible={3} executed={4} starved={5}" -f $baseline.FinalK, $ablated.FinalK, $delta, $polishEligible, $polishExecuted, $starved)
        }
    }
}

$rows | Export-Csv -Path $OutputFile -NoTypeInformation -Encoding UTF8

Write-Host ""
Write-Host "=== SWEEP COMPLETE ==="
Write-Host "rows=$($rows.Count) output=$OutputFile"

Write-Host ""
Write-Host "SummaryByGraphBudget:"
$rows |
    Group-Object Graph, BudgetMs |
    ForEach-Object {
        $g = $_.Group
        [pscustomobject]@{
            Graph = [string]$g[0].Graph
            BudgetMs = [int]$g[0].BudgetMs
            Samples = [int]$g.Count
            MedianDeltaK = [int64]((@($g.PairedDeltaK | Sort-Object))[[int][math]::Floor($g.Count / 2)])
            AvgDeltaK = [math]::Round((($g | Measure-Object PairedDeltaK -Average).Average), 3)
            MaxAbsDeltaK = [int64]([math]::Max([math]::Abs(($g | Measure-Object PairedDeltaK -Minimum).Minimum), [math]::Abs(($g | Measure-Object PairedDeltaK -Maximum).Maximum)))
            ZeroDeltaRuns = [int](@($g | Where-Object { [int64]$_.PairedDeltaK -eq 0 }).Count)
            EligibleRuns = [int](@($g | Where-Object { $_.PolishEligible }).Count)
            ExecutedRuns = [int](@($g | Where-Object { $_.PolishExecuted }).Count)
            StarvedRuns = [int](@($g | Where-Object { $_.Starved }).Count)
        }
    } |
    Sort-Object Graph, BudgetMs |
    Format-Table -AutoSize
