param(
    [string[]]$CsvFiles = @(),
    [string]$CsvGlob = "run-matrix-results/matrix-parallel-1thread-*.csv",
    [string[]]$BaselineCsvFiles = @(),
    [string]$BaselineCsv = "",
    [ValidateSet("fixed", "variance", "hybrid")]
    [string]$CapMode = "hybrid",
    [double]$BaselineSpreadAlpha = 0.5,
    [double]$LargeRegressionCapPct = 1.5,
    [double]$SmallGraphMaxRegressionPct = 10.0,
    [int64]$LargeGraphKThreshold = 2000,
    [int64]$AbsoluteUnitFloor = 1,
    [int64]$SmallGraphHardCeilingClamp = 3,
    [int64]$SpreadDenominatorFloor = 10,
    [switch]$RequireSeedSetMatch,
    [switch]$GateOnSpread,
    [switch]$LatestThree
)

$ErrorActionPreference = "Stop"

function Get-GraphName {
    param([string]$Path)
    return [System.IO.Path]::GetFileNameWithoutExtension($Path)
}

function Get-Int64FromPipeline {
    param(
        [string]$Pipeline,
        [string]$Key
    )

    $m = [regex]::Match($Pipeline, [string]::Format("{0}=(\d+)", [regex]::Escape($Key)))
    if (-not $m.Success) {
        return 0L
    }
    return [int64]$m.Groups[1].Value
}

function Get-RowMetrics {
    param(
        [pscustomobject]$Row,
        [string]$RunId
    )

    $pipeline = [string]$Row.PipelineLines
    $seedValue = 0L
    if ($null -ne $Row.PSObject.Properties["Seed"] -and -not [string]::IsNullOrWhiteSpace([string]$Row.Seed)) {
        try {
            $seedValue = [int64]$Row.Seed
        } catch {
            $seedValue = 0L
        }
    }

    [pscustomobject]@{
        RunId = $RunId
        BudgetMs = [int]$Row.BudgetMs
        Graph = Get-GraphName $Row.File
        Seed = $seedValue
        ExitCode = [int]$Row.ExitCode
        FinalK = Get-Int64FromPipeline -Pipeline $pipeline -Key "final_k"
        FinalCrossings = Get-Int64FromPipeline -Pipeline $pipeline -Key "final_crossings"
        FinalLp = Get-Int64FromPipeline -Pipeline $pipeline -Key "final_lp"
        Saturated = [int](Get-Int64FromPipeline -Pipeline $pipeline -Key "saturated")
    }
}

function Get-MedianInt64 {
    param([int64[]]$Values)
    if ($null -eq $Values -or $Values.Count -eq 0) {
        return 0L
    }

    $sorted = $Values | Sort-Object
    $n = $sorted.Count
    if (($n % 2) -eq 1) {
        return [int64]$sorted[[int](($n - 1) / 2)]
    }

    $a = [int64]$sorted[[int]($n / 2) - 1]
    $b = [int64]$sorted[[int]($n / 2)]
    return [int64][math]::Floor((($a + $b) / 2.0))
}

function Get-QuartileMetrics {
    param([int64[]]$Values)

    if ($null -eq $Values -or $Values.Count -eq 0) {
        return [pscustomobject]@{
            Q1 = 0L
            Q3 = 0L
            IQR = 0L
            Min = 0L
            Max = 0L
            Spread = 0L
        }
    }

    $sorted = @($Values | Sort-Object)
    $n = $sorted.Count
    $min = [int64]$sorted[0]
    $max = [int64]$sorted[$n - 1]
    $spread = [int64]($max - $min)

    if ($n -lt 4) {
        return [pscustomobject]@{
            Q1 = $min
            Q3 = $max
            IQR = $spread
            Min = $min
            Max = $max
            Spread = $spread
        }
    }

    $q1Idx = [int][math]::Floor(($n - 1) * 0.25)
    $q3Idx = [int][math]::Floor(($n - 1) * 0.75)
    $q1 = [int64]$sorted[$q1Idx]
    $q3 = [int64]$sorted[$q3Idx]

    return [pscustomobject]@{
        Q1 = $q1
        Q3 = $q3
        IQR = [int64]($q3 - $q1)
        Min = $min
        Max = $max
        Spread = $spread
    }
}

function Resolve-InputFiles {
    param(
        [string[]]$ExplicitFiles,
        [string]$Glob,
        [switch]$PickLatestThree
    )

    if ($ExplicitFiles.Count -gt 0) {
        return $ExplicitFiles
    }

    $files = Get-ChildItem -Path $Glob | Sort-Object LastWriteTime
    if ($PickLatestThree) {
        return ($files | Select-Object -Last 3 | ForEach-Object { $_.FullName })
    }

    return ($files | ForEach-Object { $_.FullName })
}

function Load-MetricsFromCsvFiles {
    param([string[]]$Files)

    $data = New-Object System.Collections.Generic.List[object]
    foreach ($file in $Files) {
        $rows = Import-Csv $file
        $runId = [System.IO.Path]::GetFileName($file)
        foreach ($row in $rows) {
            $data.Add((Get-RowMetrics -Row $row -RunId $runId))
        }
    }
    return $data.ToArray()
}

function Build-SummaryByBudgetGraph {
    param([object[]]$Rows)

    $grouped = $Rows | Group-Object BudgetMs, Graph
    $summary = foreach ($g in $grouped) {
        $kVals = @($g.Group | Select-Object -ExpandProperty FinalK)
        $cVals = @($g.Group | Select-Object -ExpandProperty FinalCrossings)
        $kMetrics = Get-QuartileMetrics -Values $kVals

        [pscustomobject]@{
            BudgetMs = [int]$g.Group[0].BudgetMs
            Graph = [string]$g.Group[0].Graph
            Samples = [int]$g.Count
            MedianK = Get-MedianInt64 -Values $kVals
            MedianCrossings = Get-MedianInt64 -Values $cVals
            KMin = [int64]$kMetrics.Min
            KMax = [int64]$kMetrics.Max
            KSpread = [int64]$kMetrics.Spread
            KIqr = [int64]$kMetrics.IQR
        }
    }

    return ($summary | Sort-Object BudgetMs, Graph)
}

$selectedFiles = Resolve-InputFiles -ExplicitFiles $CsvFiles -Glob $CsvGlob -PickLatestThree:$LatestThree
if ($selectedFiles.Count -eq 0) {
    throw "No CSV files selected."
}

$allRows = Load-MetricsFromCsvFiles -Files $selectedFiles

$exitFailures = ($allRows | Where-Object { $_.ExitCode -ne 0 }).Count
$saturatedRows = ($allRows | Where-Object { $_.Saturated -ne 0 }).Count

Write-Host "SelectedRuns=$($selectedFiles.Count)"
$selectedFiles | ForEach-Object { Write-Host ("- " + $_) }
Write-Host "Rows=$($allRows.Count) ExitFailures=$exitFailures SaturatedRows=$saturatedRows"

$summary = Build-SummaryByBudgetGraph -Rows $allRows

Write-Host ""
Write-Host "PerGraphSummary:"
$summary | Format-Table BudgetMs, Graph, Samples, MedianK, MedianCrossings, KMin, KMax, KSpread, KIqr -AutoSize

$budgetRollup = foreach ($b in ($summary | Group-Object BudgetMs | Sort-Object Name)) {
    $ks = @($b.Group | Select-Object -ExpandProperty MedianK)
    $cs = @($b.Group | Select-Object -ExpandProperty MedianCrossings)
    [pscustomobject]@{
        BudgetMs = [int]$b.Name
        Graphs = [int]$b.Count
        GlobalMedianK = Get-MedianInt64 -Values $ks
        GlobalMedianCrossings = Get-MedianInt64 -Values $cs
        WorstSpread = [int64](($b.Group | Sort-Object KSpread -Descending | Select-Object -First 1).KSpread)
        WorstIqr = [int64](($b.Group | Sort-Object KIqr -Descending | Select-Object -First 1).KIqr)
    }
}

Write-Host ""
Write-Host "BudgetRollup:"
$budgetRollup | Format-Table BudgetMs, Graphs, GlobalMedianK, GlobalMedianCrossings, WorstSpread, WorstIqr -AutoSize

if ($BaselineCsvFiles.Count -eq 0 -and -not [string]::IsNullOrWhiteSpace($BaselineCsv)) {
    $BaselineCsvFiles = @($BaselineCsv)
}

if ($BaselineCsvFiles.Count -gt 0) {
    foreach ($bf in $BaselineCsvFiles) {
        if (-not (Test-Path $bf)) {
            throw "Baseline CSV not found: $bf"
        }
    }

    $baselineRows = Load-MetricsFromCsvFiles -Files $BaselineCsvFiles
    $baselineSummary = Build-SummaryByBudgetGraph -Rows $baselineRows

    if ($RequireSeedSetMatch) {
        $candidateSeedSets = @{}
        foreach ($group in ($allRows | Group-Object BudgetMs, Graph)) {
            $key = "$($group.Group[0].BudgetMs)|$($group.Group[0].Graph)"
            $seedSet = @($group.Group | Select-Object -ExpandProperty Seed | Sort-Object -Unique)
            $candidateSeedSets[$key] = (($seedSet | ForEach-Object { $_.ToString() }) -join ",")
        }

        $baselineSeedSets = @{}
        foreach ($group in ($baselineRows | Group-Object BudgetMs, Graph)) {
            $key = "$($group.Group[0].BudgetMs)|$($group.Group[0].Graph)"
            $seedSet = @($group.Group | Select-Object -ExpandProperty Seed | Sort-Object -Unique)
            $baselineSeedSets[$key] = (($seedSet | ForEach-Object { $_.ToString() }) -join ",")
        }

        $mismatches = New-Object System.Collections.Generic.List[string]
        foreach ($key in $candidateSeedSets.Keys) {
            if (-not $baselineSeedSets.ContainsKey($key)) {
                continue
            }
            if ($candidateSeedSets[$key] -ne $baselineSeedSets[$key]) {
                $mismatches.Add("$key candidate=[$($candidateSeedSets[$key])] baseline=[$($baselineSeedSets[$key])]")
            }
        }

        if ($mismatches.Count -gt 0) {
            Write-Host ""
            Write-Host "SeedSetMismatches:"
            $mismatches | ForEach-Object { Write-Host $_ }
            throw "Seed set mismatch detected between candidate and baseline. Use paired seed portfolios."
        }
    }

    $baselineMap = @{}
    foreach ($b in $baselineSummary) {
        $key = "$($b.BudgetMs)|$($b.Graph)"
        $baselineMap[$key] = $b
    }

    $deltas = foreach ($s in $summary) {
        $key = "$($s.BudgetMs)|$($s.Graph)"
        if (-not $baselineMap.ContainsKey($key)) {
            continue
        }

        $base = $baselineMap[$key]
        $deltaK = [int64]$s.MedianK - [int64]$base.MedianK
        $deltaCross = [int64]$s.MedianCrossings - [int64]$base.MedianCrossings
        $regPct = 0.0
        if ([int64]$base.MedianK -gt 0) {
            $regPct = (100.0 * [double]$deltaK) / [double]$base.MedianK
        }

        $capAbs = 0L
        $capPct = 0.0
        $capViolated = $false
        if ($deltaK -gt 0) {
            $varLimit = [int64][math]::Ceiling([double]$base.KIqr * [double]$BaselineSpreadAlpha)
            if ($varLimit -lt 0) {
                $varLimit = 0L
            }

            $pctCeiling = 0L
            if ([int64]$base.MedianK -ge $LargeGraphKThreshold) {
                $pctCeiling = [int64][math]::Ceiling(([double]$base.MedianK * [double]$LargeRegressionCapPct) / 100.0)
            } else {
                $rawSmallCap = [int64][math]::Ceiling(([double]$base.MedianK * [double]$SmallGraphMaxRegressionPct) / 100.0)
                $pctCeiling = [int64][math]::Min($rawSmallCap, $SmallGraphHardCeilingClamp)
            }
            if ($pctCeiling -lt 0L) {
                $pctCeiling = 0L
            }

            if ($CapMode -eq "hybrid") {
                $allowed = $varLimit
                if ($pctCeiling -gt 0L -and $pctCeiling -lt $allowed) {
                    $allowed = $pctCeiling
                }
                if ($allowed -lt $AbsoluteUnitFloor) {
                    $allowed = $AbsoluteUnitFloor
                }

                $capAbs = [int64]$allowed
                if ($deltaK -gt $allowed) {
                    $capViolated = $true
                }
            } elseif ($CapMode -eq "variance") {
                $capAbs = [int64]$varLimit
                if ($deltaK -gt $varLimit) {
                    $capViolated = $true
                }
            } else {
                $capPct = ($pctCeiling * 100.0) / [double][math]::Max(1, [int64]$base.MedianK)
                if ($capPct -le 0.0 -or $regPct -gt $capPct) {
                    $capViolated = $true
                }
            }
        }

        [pscustomobject]@{
            BudgetMs = [int]$s.BudgetMs
            Graph = [string]$s.Graph
            BaselineSamples = [int]$base.Samples
            BaselineK = [int64]$base.MedianK
            MedianK = [int64]$s.MedianK
            DeltaK = [int64]$deltaK
            DeltaKPercent = [math]::Round($regPct, 4)
            BaselineCrossings = [int64]$base.MedianCrossings
            MedianCrossings = [int64]$s.MedianCrossings
            DeltaCrossings = [int64]$deltaCross
            BaselineKIqr = [int64]$base.KIqr
            BaselineKSpread = [int64]$base.KSpread
            KIqr = [int64]$s.KIqr
            KSpread = [int64]$s.KSpread
            CapMode = [string]$CapMode
            CapAbs = [int64]$capAbs
            CapPercent = [double]$capPct
            CapViolated = [bool]$capViolated
        }
    }

    $deltas = $deltas | Sort-Object BudgetMs, Graph

    Write-Host ""
    Write-Host "BaselineDeltaSummary:"
    $deltas | Format-Table BudgetMs, Graph, BaselineSamples, BaselineK, MedianK, DeltaK, DeltaKPercent, BaselineKIqr, KIqr, BaselineKSpread, KSpread, CapMode, CapAbs, CapPercent, CapViolated -AutoSize

    $policy = foreach ($bg in ($deltas | Group-Object BudgetMs | Sort-Object Name)) {
        $arr = $bg.Group
        $kDeltaVals = @($arr | Select-Object -ExpandProperty DeltaK)
        $crossDeltaVals = @($arr | Select-Object -ExpandProperty DeltaCrossings)
        $medianDelta = Get-MedianInt64 -Values $kDeltaVals
        $medianDeltaCross = Get-MedianInt64 -Values $crossDeltaVals
        $maxRegression = [int64](($arr | Sort-Object DeltaK -Descending | Select-Object -First 1).DeltaK)
        $totalNormalizedIqrDelta = 0.0
        foreach ($item in $arr) {
            $denom = [double][math]::Max([int64]$item.BaselineK, $SpreadDenominatorFloor)
            $totalNormalizedIqrDelta += (([int64]$item.KIqr - [int64]$item.BaselineKIqr) / $denom)
        }
        $totalBaselineIqr = [int64](($arr | Measure-Object -Property BaselineKIqr -Sum).Sum)
        $totalCandidateIqr = [int64](($arr | Measure-Object -Property KIqr -Sum).Sum)
        $spreadImproved = ($totalNormalizedIqrDelta -lt 0.0)
        $violations = ($arr | Where-Object { $_.CapViolated }).Count
        $primaryImproves = ($medianDelta -lt 0)
        $neutralPrimaryWithBetterSecondary = (($medianDelta -eq 0) -and ($medianDeltaCross -lt 0))
        $varianceReductionWin = (($medianDelta -eq 0) -and ($medianDeltaCross -eq 0) -and $spreadImproved -and $GateOnSpread)
        $tierPass = (($primaryImproves -or $neutralPrimaryWithBetterSecondary -or $varianceReductionWin) -and ($violations -eq 0))
        $tierNonRegressing = (($medianDelta -le 0) -and ($violations -eq 0))
        [pscustomobject]@{
            BudgetMs = [int]$bg.Name
            CapMode = [string]$CapMode
            MedianDeltaK = [int64]$medianDelta
            MedianDeltaCrossings = [int64]$medianDeltaCross
            MaxRegressionK = [int64]$maxRegression
            BaselineTotalIqr = [int64]$totalBaselineIqr
            CandidateTotalIqr = [int64]$totalCandidateIqr
            NormalizedIqrDelta = [math]::Round($totalNormalizedIqrDelta, 6)
            SpreadImproved = [bool]$spreadImproved
            CapViolations = [int]$violations
            TierPass = [bool]$tierPass
            TierNonRegressing = [bool]$tierNonRegressing
            AcceptPolicyPass = [bool]$tierPass
        }
    }

    Write-Host ""
    Write-Host "AcceptancePolicy:"
    $policy | Format-Table BudgetMs, CapMode, MedianDeltaK, MedianDeltaCrossings, MaxRegressionK, BaselineTotalIqr, CandidateTotalIqr, NormalizedIqrDelta, SpreadImproved, CapViolations, TierPass, TierNonRegressing, AcceptPolicyPass -AutoSize

    Write-Host ""
    Write-Host "MultiBudgetCascade:"
    $anyTierPassed = (($policy | Where-Object { $_.TierPass }).Count -gt 0)
    $allTiersSafe = (($policy | Where-Object { -not $_.TierNonRegressing }).Count -eq 0)
    $globalPromotionAccepted = ($anyTierPassed -and $allTiersSafe)
    Write-Host "AnyTierPassed=$anyTierPassed"
    Write-Host "AllTiersNonRegressing=$allTiersSafe"
    Write-Host "GlobalPromotionAccepted=$globalPromotionAccepted"
}
