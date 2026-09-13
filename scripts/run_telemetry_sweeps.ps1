param(
    [string]$SeedFile = "run-matrix-results/canonical-seeds-7-20260620-155939.txt",
    [string]$BinaryPath = "build-release/Release/SALAHC.exe",
    [string]$GraphPattern = "data/Automatic-*.json",
    [string[]]$Graphs = @("Automatic-8", "Automatic-9"),
    [UInt64[]]$Seeds = @(),
    [int[]]$BudgetsMs = @(20000, 60000, 300000),
    [int]$ThreadsPerRun = 1,
    [int]$JobCount = 12,
    [string]$LogsRoot = "run-matrix-results/logs"
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $repoRoot

$exeFullPath = Resolve-Path $BinaryPath -ErrorAction Stop
$graphFiles = Get-ChildItem -Path $GraphPattern | Sort-Object Name
if ($Graphs.Count -gt 0) {
    $wanted = @{}
    foreach ($g in $Graphs) { $wanted[$g] = $true }
    $graphFiles = @($graphFiles | Where-Object { $wanted.ContainsKey([System.IO.Path]::GetFileNameWithoutExtension($_.Name)) })
}
if ($graphFiles.Count -eq 0) {
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

$baselineDir = Join-Path $LogsRoot "baseline"
$ablatedDir = Join-Path $LogsRoot "ablated"
New-Item -ItemType Directory -Path $baselineDir -Force | Out-Null
New-Item -ItemType Directory -Path $ablatedDir -Force | Out-Null

$tasks = New-Object System.Collections.Generic.List[object]
foreach ($budget in $BudgetsMs) {
    foreach ($seed in $Seeds) {
        foreach ($file in $graphFiles) {
            $graphName = [System.IO.Path]::GetFileNameWithoutExtension($file.Name)
            $tasks.Add([pscustomobject]@{
                BudgetMs = [int]$budget
                Seed = [UInt64]$seed
                GraphName = $graphName
                InputPath = $file.FullName
                Mode = "baseline"
            })
            $tasks.Add([pscustomobject]@{
                BudgetMs = [int]$budget
                Seed = [UInt64]$seed
                GraphName = $graphName
                InputPath = $file.FullName
                Mode = "ablated"
            })
        }
    }
}

$maxJobs = [Math]::Min([Math]::Max(1, $JobCount), $tasks.Count)
$chunks = @()
for ($j = 0; $j -lt $maxJobs; $j++) {
    $chunks += ,(New-Object System.Collections.Generic.List[object])
}
for ($i = 0; $i -lt $tasks.Count; $i++) {
    [void]$chunks[$i % $maxJobs].Add($tasks[$i])
}

Write-Host "[telemetry-sweep] tasks=$($tasks.Count) jobs=$maxJobs graphs=$($graphFiles.Count) seeds=$($Seeds.Count) budgets=$($BudgetsMs -join ',')"

$jobs = @()
for ($j = 0; $j -lt $maxJobs; $j++) {
    $chunk = $chunks[$j].ToArray()
    $jobs += Start-Job -ScriptBlock {
        param($exePath, $taskChunk, $threadsPerRun, $baselineDirPath, $ablatedDirPath)

        # External tools may write to stderr for informational logs; do not treat that as terminating.
        $ErrorActionPreference = "Continue"

        foreach ($task in $taskChunk) {
            $logFile = "{0}_{1}_seed{2}.log" -f $task.GraphName, $task.BudgetMs, $task.Seed
            $targetDir = if ($task.Mode -eq "baseline") { $baselineDirPath } else { $ablatedDirPath }
            $logPath = Join-Path $targetDir $logFile

            $env:GDCONTESTAI_POLISH_TELEMETRY = "1"
            $env:GDCONTESTAI_TIME_LIMIT_MS = [string]$task.BudgetMs
            $env:GDCONTESTAI_THREADS = [string]$threadsPerRun
            $env:GDCONTESTAI_SEED = [string]$task.Seed
            if ($task.Mode -eq "ablated") {
                $env:GDCONTESTAI_DISABLE_FINAL_POLISH = "1"
            } else {
                $env:GDCONTESTAI_DISABLE_FINAL_POLISH = "0"
            }

            & $exePath $task.InputPath *> $logPath
            Write-Host ("[{0}] budget={1} seed={2} graph={3}" -f $task.Mode, $task.BudgetMs, $task.Seed, $task.GraphName)
        }
    } -ArgumentList $exeFullPath.Path, $chunk, $ThreadsPerRun, (Resolve-Path $baselineDir).Path, (Resolve-Path $ablatedDir).Path
}

Wait-Job -Job $jobs | Out-Null
foreach ($job in $jobs) {
    Receive-Job -Job $job
    if ($job.State -ne "Completed") {
        throw "Telemetry sweep worker failed with state $($job.State)."
    }
    Remove-Job -Job $job -Force
}

Write-Host "[telemetry-sweep] complete"
Write-Host "[telemetry-sweep] baseline_logs=$baselineDir"
Write-Host "[telemetry-sweep] ablated_logs=$ablatedDir"
