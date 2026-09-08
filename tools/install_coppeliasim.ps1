param(
    [Parameter(Mandatory = $true)]
    [string]$CoppeliaRoot,
    [ValidateSet('Debug', 'Release')]
    [string]$Configuration = 'Release'
)

$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$simRoot = (Resolve-Path -LiteralPath $CoppeliaRoot).Path
$simExe = Join-Path $simRoot 'coppeliaSim.exe'
if (-not (Test-Path -LiteralPath $simExe -PathType Leaf)) {
    throw "CoppeliaSim executable was not found: $simExe"
}

$addOnDir = Join-Path $simRoot 'addOns'
$configDir = Join-Path $simRoot 'config'
$luaDir = Join-Path $simRoot 'lua'
New-Item -ItemType Directory -Force -Path $addOnDir, $configDir, $luaDir | Out-Null

function Copy-VerifiedFile([string]$Source, [string]$DestinationDirectory) {
    Copy-Item -LiteralPath $Source -Destination $DestinationDirectory -Force
    $destination = Join-Path $DestinationDirectory (Split-Path -Leaf $Source)
    if (-not (Test-Path -LiteralPath $destination -PathType Leaf)) {
        throw "Installed file is missing after copy: $destination"
    }
    $sourceHash = (Get-FileHash -LiteralPath $Source -Algorithm SHA256).Hash
    $destinationHash = (Get-FileHash -LiteralPath $destination -Algorithm SHA256).Hash
    if ($sourceHash -ne $destinationHash) {
        throw "Installed file hash mismatch: $destination"
    }
    Write-Host "Installed: $destination"
}

Copy-VerifiedFile (Join-Path $projectRoot 'coppeliasim\simAddOnRep5xSceneBuilder.lua') $addOnDir
Copy-VerifiedFile (Join-Path $projectRoot 'coppeliasim\rep5x_profile.lua') $addOnDir
Copy-VerifiedFile (Join-Path $projectRoot 'coppeliasim\simExtFIBR3D.lua') $luaDir
Copy-VerifiedFile (Join-Path $projectRoot 'config\rep5x_ender3_v3_se.json') $configDir

$pluginCandidates = @(
    (Join-Path $projectRoot "v-rep_plugin\build-rep5x-msvc\$Configuration\simExtFIBR3D.dll"),
    (Join-Path $projectRoot "v-rep_plugin\build-rep5x\$Configuration\simExtFIBR3D.dll")
)
$plugin = $pluginCandidates | Where-Object { Test-Path -LiteralPath $_ -PathType Leaf } | Select-Object -First 1
if ($plugin) {
    Copy-VerifiedFile $plugin $simRoot
    Get-ChildItem -LiteralPath (Split-Path -Parent $plugin) -Filter 'boost_*.dll' -File | ForEach-Object {
        Copy-VerifiedFile $_.FullName $simRoot
    }
} else {
    Write-Warning 'No freshly built Rep5x simExtFIBR3D.dll was found. The checked-in upstream DLL is intentionally not installed because it does not contain the Rep5x adapter. Local scene/jog/G-code controls work without the plugin.'
}

Write-Host 'Restart CoppeliaSim, open Add-ons > Rep5x Scene Builder, and click Build scene.'
