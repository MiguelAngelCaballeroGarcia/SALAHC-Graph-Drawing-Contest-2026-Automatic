param(
    [ValidateSet("contest", "parallel-1thread")]
    [string]$Mode = "contest",

    [int[]]$BudgetsMs = @(20000, 60000, 300000),

    [string]$IncludePattern = "data/Automatic-*.json",

    [int]$ThreadsPerRun = 0,

    [int]$MaxParallel = 0,

    [int]$OSHeadroom = 2,

    [UInt64]$Seed = 0,

    [UInt64[]]$Seeds = @(),

    [switch]$UseCanonicalSeeds,

    [switch]$BuildFirst
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $repoRoot

$exePath = Join-Path $repoRoot "build-release/Release/SALAHC.exe"
if (-not (Test-Path $exePath)) {
    throw "Executable not found at $exePath. Build first or pass -BuildFirst."
}

if ($BuildFirst) {
    Write-Host "[build] cmake --build build-release --config Release --target SALAHC"
    cmd /c "cmake --build build-release --config Release --target SALAHC"
    if ($LASTEXITCODE -ne 0) {
        throw "Build failed with exit code $LASTEXITCODE"
    }
}

$files = Get-ChildItem -Path $IncludePattern | Sort-Object Name | ForEach-Object { $_.FullName }
if ($files.Count -eq 0) {
    throw "No input files matched pattern: $IncludePattern"
}

if ($Mode -eq "parallel-1thread") {
    if ($ThreadsPerRun -le 0) {
        $ThreadsPerRun = 1
    }
    if ($MaxParallel -le 0) {
        $logicalThreads = [Math]::Max(1, [Environment]::ProcessorCount)
        $safeParallel = [Math]::Max(1, $logicalThreads - [Math]::Max(0, $OSHeadroom))
        $MaxParallel = [Math]::Min($files.Count, $safeParallel)
    }
} else {
    if ($ThreadsPerRun -lt 0) {
        throw "ThreadsPerRun must be >= 0"
    }
    if ($MaxParallel -le 0) {
        $MaxParallel = 1
    }
}

Write-Host "[config] mode=$Mode budgets=$($BudgetsMs -join ',') files=$($files.Count) threads-per-run=$ThreadsPerRun max-parallel=$MaxParallel os-headroom=$OSHeadroom"

if ($Seed -gt 0 -and $Seeds.Count -gt 0) {
    throw "Use either -Seed or -Seeds, not both."
}

if ($UseCanonicalSeeds -and $Seeds.Count -gt 0) {
    throw "Use either -UseCanonicalSeeds or -Seeds, not both."
}

$seedList = @()
if ($Seeds.Count -gt 0) {
    $seedList = @($Seeds)
} elseif ($UseCanonicalSeeds) {
    $seedList = @(1337, 4242, 7201, 9109, 11047, 13003, 15427)
} else {
    $seedList = @($Seed)
}

if ($seedList.Count -eq 0) {
    $seedList = @(0)
}

$seedLabel = ($seedList | ForEach-Object { $_.ToString() }) -join ","
if ($seedList.Count -eq 1) {
    Write-Host "[config] seed=$seedLabel"
} else {
    Write-Host "[config] seeds=$seedLabel"
}

function Invoke-OneRun {
    param(
        [string]$Exe,
        [int]$Budget,
        [string]$InputFile,
        [int]$Threads,
        [UInt64]$SeedValue
    )

    $parts = @("set GDCONTESTAI_TIME_LIMIT_MS=$Budget")
    if ($Threads -gt 0) {
        $parts += "set GDCONTESTAI_THREADS=$Threads"
    }
    if ($SeedValue -gt 0) {
        $parts += "set GDCONTESTAI_SEED=$SeedValue"
    }
    $parts += "`"$Exe`" `"$InputFile`""
    $cmd = [string]::Join(" && ", $parts)

    $output = cmd /c $cmd 2>&1 | Out-String
    $exitCode = $LASTEXITCODE

    [pscustomobject]@{
        BudgetMs = $Budget
        File = $InputFile
        Threads = $Threads
        Seed = $SeedValue
        ExitCode = $exitCode
        Output = $output
    }
}

function Print-RunResult {
    param([pscustomobject]$Result)

    $name = Split-Path $Result.File -Leaf
    Write-Host "### budget=$($Result.BudgetMs) file=$name threads=$($Result.Threads) seed=$($Result.Seed) exit=$($Result.ExitCode)"

    $summary = Get-PipelineLines -OutputText $Result.Output

    if ($summary.Count -eq 0) {
        Write-Host "(no pipeline lines captured)"
    } else {
        $summary | ForEach-Object { Write-Host $_ }
    }
}

function Get-PipelineLines {
    param([string]$OutputText)

    if ([string]::IsNullOrWhiteSpace($OutputText)) {
        return @()
    }

    # Console wrapping can split long PIPELINE lines across multiple physical lines.
    # Normalize whitespace and recover each full marker segment.
    $normalized = ($OutputText -replace "`r?`n", " ") -replace "\s+", " "

    $results = New-Object System.Collections.Generic.List[string]
    $patterns = @(
        '\[PIPELINE\]\[budgets\].*?(?=\[PIPELINE\]\[summary\]|$)',
        '\[PIPELINE\]\[summary\].*?(?=\[PIPELINE\]\[budgets\]|$)'
    )

    foreach ($p in $patterns) {
        foreach ($m in [regex]::Matches($normalized, $p)) {
            $line = $m.Value.Trim()
            if (-not [string]::IsNullOrWhiteSpace($line)) {
                $results.Add($line)
            }
        }
    }

    return $results
}

$results = New-Object System.Collections.Generic.List[object]

if ($Mode -eq "contest") {
    foreach ($budget in $BudgetsMs) {
        foreach ($seedValue in $seedList) {
            foreach ($file in $files) {
                $result = Invoke-OneRun -Exe $exePath -Budget $budget -InputFile $file -Threads $ThreadsPerRun -SeedValue $seedValue
                $results.Add($result)
                Print-RunResult -Result $result
            }
        }
    }
} else {
    $tasks = New-Object System.Collections.Generic.List[object]
    foreach ($budget in $BudgetsMs) {
        foreach ($seedValue in $seedList) {
            foreach ($file in $files) {
                $tasks.Add([pscustomobject]@{ BudgetMs = $budget; File = $file; Seed = $seedValue })
            }
        }
    }

    $running = New-Object System.Collections.Generic.List[object]
    $next = 0

    while ($next -lt $tasks.Count -or $running.Count -gt 0) {
        while ($running.Count -lt $MaxParallel -and $next -lt $tasks.Count) {
            $t = $tasks[$next]
            $job = Start-Job -ScriptBlock {
                param($exe, $budget, $file, $threads, $seed)

                $parts = @("set GDCONTESTAI_TIME_LIMIT_MS=$budget", "set GDCONTESTAI_THREADS=$threads")
                if ($seed -gt 0) {
                    $parts += "set GDCONTESTAI_SEED=$seed"
                }
                $parts += "`"$exe`" `"$file`""
                $cmd = [string]::Join(" && ", $parts)
                $output = cmd /c $cmd 2>&1 | Out-String
                $exitCode = $LASTEXITCODE

                [pscustomobject]@{
                    BudgetMs = $budget
                    File = $file
                    Threads = $threads
                    Seed = $seed
                    ExitCode = $exitCode
                    Output = $output
                }
            } -ArgumentList $exePath, $t.BudgetMs, $t.File, $ThreadsPerRun, $t.Seed

            $running.Add($job)
            $next += 1
        }

        $finished = Wait-Job -Job $running -Any -Timeout 1
        if ($null -ne $finished) {
            $result = Receive-Job -Job $finished
            $results.Add($result)
            Print-RunResult -Result $result

            Remove-Job -Job $finished -Force
            [void]$running.Remove($finished)
        }
    }
}

$timestamp = Get-Date -Format "yyyyMMdd-HHmmss"
$outDir = Join-Path $repoRoot "run-matrix-results"
New-Item -ItemType Directory -Path $outDir -Force | Out-Null
$outPath = Join-Path $outDir "matrix-$Mode-$timestamp.csv"

$results |
    Select-Object BudgetMs, File, Threads, Seed, ExitCode,
        @{ Name = "PipelineLines"; Expression = { (Get-PipelineLines -OutputText $_.Output) -join " || " } } |
    Export-Csv -Path $outPath -NoTypeInformation -Encoding UTF8

Write-Host "Saved matrix results to $outPath"
