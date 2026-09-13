param(
    [string]$BaselineCsv = "run-matrix-results/matrix-parallel-1thread-20260620-155910.csv",
    [string]$NoPolishCsv = "run-matrix-results/canonical-no-polish-full-sweep.csv",
    [string]$OutCsv = "run-matrix-results/polish-diagnostics.csv"
)

$ErrorActionPreference = "Stop"

if (-not (Test-Path $BaselineCsv)) {
    throw "Baseline CSV not found: $BaselineCsv"
}
if (-not (Test-Path $NoPolishCsv)) {
    throw "No-polish CSV not found: $NoPolishCsv"
}

function Get-Int64FromPipeline {
    param(
        [string]$Pipeline,
        [string]$Key
    )

    if ([string]::IsNullOrWhiteSpace($Pipeline)) {
        return 0L
    }

    $m = [regex]::Match($Pipeline, [string]::Format("{0}=(\d+)", [regex]::Escape($Key)))
    if (-not $m.Success) {
        return 0L
    }

    return [int64]$m.Groups[1].Value
}

function Normalize-GraphName {
    param([string]$Path)

    if ([string]::IsNullOrWhiteSpace($Path)) {
        return ""
    }

    return [System.IO.Path]::GetFileNameWithoutExtension($Path)
}

$baseRows = Import-Csv $BaselineCsv
$candRows = Import-Csv $NoPolishCsv

$baseIndex = @{}
foreach ($row in $baseRows) {
    $graph = Normalize-GraphName -Path $row.File
    $budget = [int]$row.BudgetMs
    $seed = [int64]$row.Seed
    $key = "$graph|$budget|$seed"

    $baseIndex[$key] = [pscustomobject]@{
        Graph = $graph
        BudgetMs = $budget
        Seed = $seed
        ExitCode = [int]$row.ExitCode
        FinalK = Get-Int64FromPipeline -Pipeline ([string]$row.PipelineLines) -Key "final_k"
    }
}

$diagnostics = New-Object System.Collections.Generic.List[object]
$unpaired = 0
foreach ($row in $candRows) {
    $graph = Normalize-GraphName -Path $row.File
    $budget = [int]$row.BudgetMs
    $seed = [int64]$row.Seed
    $key = "$graph|$budget|$seed"

    if (-not $baseIndex.ContainsKey($key)) {
        ++$unpaired
        continue
    }

    $base = $baseIndex[$key]
    $candFinalK = Get-Int64FromPipeline -Pipeline ([string]$row.PipelineLines) -Key "final_k"
    $deltaK = [int64]($candFinalK - $base.FinalK)

    $status = "Neutral"
    if ($deltaK -gt 0) {
        $status = "Polish_Beneficial"
    } elseif ($deltaK -lt 0) {
        $status = "Polish_Harmful"
    }

    $diagnostics.Add([pscustomobject]@{
        Graph = $graph
        BudgetMs = $budget
        Seed = $seed
        BaselineFinalK = [int64]$base.FinalK
        NoPolishFinalK = [int64]$candFinalK
        DeltaK = $deltaK
        Status = $status
        BaselineExitCode = [int]$base.ExitCode
        NoPolishExitCode = [int]$row.ExitCode
    })
}

$diagnostics | Export-Csv -Path $OutCsv -NoTypeInformation -Encoding UTF8

Write-Host "PairedRows=$($diagnostics.Count) UnpairedRows=$unpaired"
Write-Host "Saved diagnostic rows to $OutCsv"

Write-Host ""
Write-Host "DecisionMatrixByGraphBudget:"
$diagnostics |
    Group-Object Graph, BudgetMs |
    ForEach-Object {
        $rows = $_.Group
        [pscustomobject]@{
            Graph = [string]$rows[0].Graph
            BudgetMs = [int]$rows[0].BudgetMs
            Samples = [int]$rows.Count
            AvgDeltaK = [math]::Round((($rows | Measure-Object DeltaK -Average).Average), 3)
            BeneficialRuns = [int](($rows | Where-Object { $_.Status -eq "Polish_Beneficial" }).Count)
            HarmfulRuns = [int](($rows | Where-Object { $_.Status -eq "Polish_Harmful" }).Count)
            NeutralRuns = [int](($rows | Where-Object { $_.Status -eq "Neutral" }).Count)
            MinDeltaK = [int64](($rows | Measure-Object DeltaK -Minimum).Minimum)
            MaxDeltaK = [int64](($rows | Measure-Object DeltaK -Maximum).Maximum)
        }
    } |
    Sort-Object BudgetMs, Graph |
    Format-Table -AutoSize
