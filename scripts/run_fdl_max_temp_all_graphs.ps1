param(
    [string]$InputPattern = "Automatic-*.json",
    [string]$BuildDir = "build-clean",
    [ValidateSet("Release", "Debug")]
    [string]$Config = "Release",
    [int]$TimeLimitMs = 120000,
    [object]$NoTimeLimit = $true,
    [double]$CoolingFactor = 0.99995,
    [switch]$BuildFirst,
    [switch]$ClearOutput
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $repoRoot

function Convert-ToBoolean {
    param(
        [Parameter(Mandatory = $true)]
        [object]$Value,
        [bool]$DefaultValue = $false
    )

    if ($null -eq $Value) {
        return $DefaultValue
    }

    if ($Value -is [bool]) {
        return [bool]$Value
    }

    if ($Value -is [byte] -or $Value -is [int16] -or $Value -is [int32] -or
        $Value -is [int64] -or $Value -is [uint16] -or $Value -is [uint32] -or
        $Value -is [uint64] -or $Value -is [double] -or $Value -is [single]) {
        return ([double]$Value) -ne 0.0
    }

    $text = [string]$Value
    if ([string]::IsNullOrWhiteSpace($text)) {
        return $DefaultValue
    }

    $normalized = $text.Trim().ToLowerInvariant()
    switch ($normalized) {
        "1" { return $true }
        "0" { return $false }
        "true" { return $true }
        "false" { return $false }
        "$true" { return $true }
        "$false" { return $false }
        "yes" { return $true }
        "no" { return $false }
        "on" { return $true }
        "off" { return $false }
        default {
            throw "Invalid NoTimeLimit value '$Value'. Use true/false (or 1/0)."
        }
    }
}

$NoTimeLimit = Convert-ToBoolean -Value $NoTimeLimit -DefaultValue $true

if ($BuildFirst) {
    Write-Host "[build] cmake --build $BuildDir --config $Config --target SALAHC"
    cmd /c "cmake --build $BuildDir --config $Config --target SALAHC"
    if ($LASTEXITCODE -ne 0) {
        throw "Build failed with exit code $LASTEXITCODE"
    }
}

$exePath = Join-Path $repoRoot "$BuildDir/$Config/SALAHC.exe"
if (-not (Test-Path $exePath)) {
    throw "Executable not found at $exePath"
}

$dataDir = Join-Path $repoRoot "data"
if (-not (Test-Path $dataDir)) {
    throw "Data directory not found: $dataDir"
}

$inputs = Get-ChildItem -Path $dataDir -Filter $InputPattern -File | Sort-Object Name
if ($inputs.Count -eq 0) {
    throw "No input files matched pattern '$InputPattern' in $dataDir"
}

function Get-GraphFdlInitialTemperature {
    param([string]$FilePath)

    $raw = Get-Content -Path $FilePath -Raw
    $json = $raw | ConvertFrom-Json

    $nodeCount = 1
    if ($null -ne $json.nodes) {
        $nodeCount = [Math]::Max(1, @($json.nodes).Count)
    }

    $width = 0.0
    $height = 0.0
    if ($null -ne $json.width) {
        $width = [double]$json.width
    }
    if ($null -ne $json.height) {
        $height = [double]$json.height
    }

    $k = [Math]::Sqrt((($width + 1.0) * ($height + 1.0)) / [double]$nodeCount)
    if ($k -lt 1.0) {
        $k = 1.0
    }

    $diag = [Math]::Sqrt(($width * $width) + ($height * $height))
    $maxDisplacement = [Math]::Max(1.0, 0.05 * $diag)

    # Match ForceDirectedLayout::run effective initial temperature after clamp.
    return [Math]::Min($k, $maxDisplacement)
}

$maxTemp = 0.0
foreach ($input in $inputs) {
    $temp = Get-GraphFdlInitialTemperature -FilePath $input.FullName
    if ($temp -gt $maxTemp) {
        $maxTemp = $temp
    }
}

if ($maxTemp -le 0.0) {
    throw "Computed max FDL temperature is invalid: $maxTemp"
}

$maxTempText = $maxTemp.ToString("R", [System.Globalization.CultureInfo]::InvariantCulture)

$fallbackOutputDir = Join-Path $dataDir "solutions"
$outDir = Join-Path $dataDir "FDL"
New-Item -ItemType Directory -Path $outDir -Force | Out-Null
if ($ClearOutput) {
    Get-ChildItem -Path $outDir -Filter *.json -File -ErrorAction SilentlyContinue | Remove-Item -Force
}

Write-Host "[config] inputs=$($inputs.Count) fixed_fdl_temperature=$maxTempText time_limit_ms=$TimeLimitMs"
Write-Host "[config] output_dir=$outDir"
Write-Host "[config] one_thread_per_graph=1 (single process, concurrent graphs)"
Write-Host "[config] fdl_cooling_factor=$CoolingFactor"

$env:GDCONTESTAI_FDL_ONLY = "1"
$env:GDCONTESTAI_FDL_FIXED_TEMPERATURE = $maxTempText
$env:GDCONTESTAI_FDL_COOLING_FACTOR = $CoolingFactor.ToString("R", [System.Globalization.CultureInfo]::InvariantCulture)
$env:GDCONTESTAI_THREADS = [string]$inputs.Count

if ($NoTimeLimit) {
    $env:GDCONTESTAI_NO_TIME_LIMIT = "1"
    Write-Host "[config] no_time_limit=1"
} else {
    Remove-Item Env:GDCONTESTAI_NO_TIME_LIMIT -ErrorAction SilentlyContinue
    $env:GDCONTESTAI_TIME_LIMIT_MS = [string]$TimeLimitMs
    Write-Host "[config] no_time_limit=0"
}

Write-Host "[run] launching all graphs in one process"
& $exePath @($inputs.FullName)
if ($LASTEXITCODE -ne 0) {
    throw "Execution failed with exit code $LASTEXITCODE"
}

foreach ($input in $inputs) {
    $producedPath = Join-Path $fallbackOutputDir $input.Name
    if (-not (Test-Path $producedPath)) {
        throw "Expected output file not found: $producedPath"
    }

    $targetPath = Join-Path $outDir $input.Name
    Copy-Item -Path $producedPath -Destination $targetPath -Force
}

Write-Host "[done] Saved FDL-only outputs to $outDir"
Write-Host "[done] Fixed FDL temperature used: $maxTempText"
