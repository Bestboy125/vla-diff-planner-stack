param(
    [string]$OpenVlaRoot = "E:\embodied_agent\uav-flow\UAV-Flow-main\UAV-Flow-main\OpenVLA-UAV",
    [string]$OpenVlaModel = "E:\embodied_agent\uav-flow\epoch-2+b8+lr-0.0005+lora-r32+dropout-0.0--real-3ep",
    [string]$PiCheckpoint = "E:\embodied_agent\uav-flow\pi05_uav_flow_lora_ep1"
)

$ErrorActionPreference = "Stop"
$preflight = [ordered]@{}

$gpuLine = & nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader,nounits
$preflight.gpu = $gpuLine

$venvPython = Join-Path $OpenVlaRoot ".venv\Scripts\python.exe"
$pythonProbe = & $venvPython -c "import json, importlib.util, torch, transformers; print(json.dumps({'torch': torch.__version__, 'transformers': transformers.__version__, 'cuda': torch.cuda.is_available(), 'openpi': importlib.util.find_spec('openpi') is not None, 'jax': importlib.util.find_spec('jax') is not None, 'orbax': importlib.util.find_spec('orbax') is not None}))"
$preflight.python = $pythonProbe | ConvertFrom-Json

$modelConfigPath = Join-Path $OpenVlaModel "config.json"
$modelConfig = Get-Content -Raw -LiteralPath $modelConfigPath | ConvertFrom-Json
$realStats = $modelConfig.norm_stats.real.action
$preflight.openvla = [ordered]@{
    model_exists = Test-Path -LiteralPath $OpenVlaModel
    unnorm_key = "real"
    action_dimensions = $realStats.q01.Count
    q01 = $realStats.q01
    q99 = $realStats.q99
}

$piFiles = Get-ChildItem -LiteralPath $PiCheckpoint -Recurse -File
$piParent = Split-Path $PiCheckpoint -Parent
$piSourceCandidates = @(
    (Join-Path $piParent "openpi"),
    (Join-Path $piParent "OpenPI"),
    (Join-Path $piParent "openpi-main")
)
$piSourcePresent = [bool]($piSourceCandidates | Where-Object { Test-Path -LiteralPath $_ })
$preflight.pi05 = [ordered]@{
    checkpoint_exists = Test-Path -LiteralPath $PiCheckpoint
    file_count = $piFiles.Count
    total_gib = [math]::Round(($piFiles | Measure-Object Length -Sum).Sum / 1GB, 3)
    custom_config_source_present = $piSourcePresent
}

$networkInterfaces = Get-NetIPAddress -AddressFamily IPv4 |
    Where-Object { $_.IPAddress -notlike "127.*" -and $_.AddressState -eq "Preferred" } |
    ForEach-Object {
        [ordered]@{
            interface = $_.InterfaceAlias
            address = $_.IPAddress
            prefix_length = $_.PrefixLength
        }
    }
$preflight.network_interfaces = @($networkInterfaces)

$portResults = foreach ($portNumber in @(5007, 50051, 8554)) {
    $listener = [System.Net.Sockets.TcpListener]::new([System.Net.IPAddress]::Any, $portNumber)
    try {
        $listener.Start()
        [ordered]@{ port = $portNumber; bind = "ok" }
    }
    catch {
        [ordered]@{ port = $portNumber; bind = "in_use"; error = $_.Exception.Message }
    }
    finally {
        $listener.Stop()
    }
}
$preflight.ports = @($portResults)

$toolResults = foreach ($toolName in @("ffmpeg", "ffprobe", "gst-launch-1.0")) {
    $toolCommand = Get-Command $toolName -ErrorAction SilentlyContinue
    [ordered]@{
        name = $toolName
        available = [bool]$toolCommand
        path = if ($toolCommand) { $toolCommand.Source } else { $null }
    }
}
$preflight.media_tools = @($toolResults)

$preflight | ConvertTo-Json -Depth 8
