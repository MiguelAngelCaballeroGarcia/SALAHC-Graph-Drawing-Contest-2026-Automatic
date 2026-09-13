param(
    [int[]]$BudgetsMs = @(20000, 60000, 300000),
    [string]$IncludePattern = "data/Automatic-*.json",
    [UInt64[]]$Seeds = @(1337, 4242, 7201, 9109, 11047, 13003, 15427),
    [int]$JobCount = 12
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $repoRoot

$exePath = Join-Path $repoRoot "build-release/Release/SALAHC.exe"
if (-not (Test-Path $exePath)) {
    throw "Executable not found at $exePath"
}

$files = Get-ChildItem -Path $IncludePattern | Sort-Object Name | ForEach-Object { $_.FullName }
if ($files.Count -eq 0) {
    throw "No input files matched pattern: $IncludePattern"
}

function Get-PipelineLines {
    param([string]$OutputText)

    if ([string]::IsNullOrWhiteSpace($OutputText)) {
        return @()
    }

    $normalized = ($OutputText -replace "`r?`n", " ") -replace "\s+", " "
    $results = New-Object System.Collections.Generic.List[string]
    $patterns = @(
        '\[PIPELINE\]\[budgets\].*?(?=\[PIPELINE\]\[summary\]|$)',
        '\[PIPELINE\]\[summary\].*?(?=\[PIPELINE\]\[budgets\]|$)'
    )

    foreach ($pattern in $patterns) {
        foreach ($match in [regex]::Matches($normalized, $pattern)) {
            $line = $match.Value.Trim()
            if (-not [string]::IsNullOrWhiteSpace($line)) {
                $results.Add($line)
            }
        }
    }

    return $results
}

$tasks = New-Object System.Collections.Generic.List[object]
foreach ($budget in $BudgetsMs) {
    foreach ($seed in $Seeds) {
        foreach ($file in $files) {
            $tasks.Add([pscustomobject]@{
                BudgetMs = [int]$budget
                Seed = [UInt64]$seed
                File = $file
            })
        }
    }
}

$jobCount = [Math]::Min([Math]::Max(1, $JobCount), $tasks.Count)
$chunks = for ($j = 0; $j -lt $jobCount; $j++) {
    New-Object System.Collections.Generic.List[object]
}
for ($i = 0; $i -lt $tasks.Count; $i++) {
    [void]$chunks[$i % $jobCount].Add($tasks[$i])
}

$jobs = @()
for ($j = 0; $j -lt $jobCount; $j++) {
    $chunk = $chunks[$j].ToArray()
    $jobs += Start-Job -ScriptBlock {
        param($exePath, $taskChunk, $jobIndex, $repoPath)

        function Get-PipelineLines {
            param([string]$OutputText)

            if ([string]::IsNullOrWhiteSpace($OutputText)) {
                return @()
            }

            $normalized = ($OutputText -replace "`r?`n", " ") -replace "\s+", " "
            $results = New-Object System.Collections.Generic.List[string]
            $patterns = @(
                '\[PIPELINE\]\[budgets\].*?(?=\[PIPELINE\]\[summary\]|$)',
                '\[PIPELINE\]\[summary\].*?(?=\[PIPELINE\]\[budgets\]|$)'
            )

            foreach ($pattern in $patterns) {
                foreach ($match in [regex]::Matches($normalized, $pattern)) {
                    $line = $match.Value.Trim()
                    if (-not [string]::IsNullOrWhiteSpace($line)) {
                        $results.Add($line)
                    }
                }
            }

            return $results
        }

        $results = New-Object System.Collections.Generic.List[object]
        $counter = 0
        foreach ($task in $taskChunk) {
            ++$counter
            Write-Host ("[job {0}] {1}/{2} budget={3} seed={4} file={5}" -f $jobIndex, $counter, $taskChunk.Count, $task.BudgetMs, $task.Seed, (Split-Path $task.File -Leaf))

            $tmp = Join-Path $env:TEMP ([guid]::NewGuid().ToString() + ".txt")
            $cmd = 'set GDCONTESTAI_DISABLE_FINAL_POLISH=1 && set GDCONTESTAI_TIME_LIMIT_MS=' + $task.BudgetMs + ' && set GDCONTESTAI_THREADS=1 && set GDCONTESTAI_SEED=' + $task.Seed + ' && "' + $exePath + '" "' + $task.File + '" > "' + $tmp + '" 2>&1'
            cmd /c $cmd | Out-Null
            $output = if (Test-Path $tmp) { Get-Content $tmp -Raw } else { '' }
            Remove-Item $tmp -ErrorAction SilentlyContinue
            $lines = @(Get-PipelineLines -OutputText $output)

            $results.Add([pscustomobject]@{
                BudgetMs = $task.BudgetMs
                File = $task.File
                Threads = 1
                Seed = [UInt64]$task.Seed
                ExitCode = $LASTEXITCODE
                PipelineLines = ($lines -join ' || ')
            })
        }

        $chunkPath = Join-Path $repoPath ("run-matrix-results/chunk-no-polish-{0}.csv" -f $jobIndex)
        $results | Export-Csv -Path $chunkPath -NoTypeInformation -Encoding UTF8
        Write-Output $chunkPath
    } -ArgumentList $exePath, $chunk, $j, $repoRoot
}

Wait-Job -Job $jobs | Out-Null
$chunkFiles = @()
foreach ($job in $jobs) {
    $chunkFiles += Receive-Job -Job $job
    Remove-Job -Job $job -Force
}

$merged = @()
foreach ($chunkFile in $chunkFiles) {
    if (Test-Path $chunkFile) {
        $merged += Import-Csv $chunkFile
    }
}

$outPath = Join-Path $repoRoot "run-matrix-results/canonical-no-polish-full-sweep.csv"
$merged | Export-Csv -Path $outPath -NoTypeInformation -Encoding UTF8
Write-Host $outPath
