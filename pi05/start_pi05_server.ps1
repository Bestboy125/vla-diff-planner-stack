$ErrorActionPreference = "Stop"

$wslDistribution = if ($env:PI05_WSL_DISTRO) { $env:PI05_WSL_DISTRO } else { "Ubuntu-22.04" }
$wslUser = $env:PI05_WSL_USER
$openPiRoot = $env:PI05_OPENPI_ROOT
$checkpointPath = $env:PI05_CHECKPOINT_PATH
$policyConfig = if ($env:PI05_POLICY_CONFIG) { $env:PI05_POLICY_CONFIG } else { "pi05_uav_flow_lora" }

foreach ($required in @{
    PI05_WSL_USER = $wslUser
    PI05_OPENPI_ROOT = $openPiRoot
    PI05_CHECKPOINT_PATH = $checkpointPath
}.GetEnumerator()) {
    if (-not $required.Value -or $required.Value -eq "REQUIRED") {
        throw "Set $($required.Key) in the private local environment."
    }
}

$command = @"
set -e
cd '$openPiRoot'
export PATH="`$HOME/.local/bin:`$PATH"
export XLA_PYTHON_CLIENT_PREALLOCATE=false
export JAX_COMPILATION_CACHE_DIR="`$HOME/.cache/jax"
exec uv run scripts/serve_policy.py \
  --port 8000 \
  policy:checkpoint \
  --policy.config='$policyConfig' \
  --policy.dir='$checkpointPath'
"@

Write-Host "Starting π0.5 OpenPI server on ws://127.0.0.1:8000"
Write-Host "Checkpoint is configured through PI05_CHECKPOINT_PATH."
wsl -d $wslDistribution -u $wslUser -- bash -lc $command
