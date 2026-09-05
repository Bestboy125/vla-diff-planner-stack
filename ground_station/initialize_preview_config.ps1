param(
    [Parameter(Mandatory = $true)]
    [ValidateNotNullOrEmpty()]
    [string]$HostOnboardIp,

    [Parameter(Mandatory = $true)]
    [ValidateNotNullOrEmpty()]
    [string]$HostOperatorIp,

    [Parameter(Mandatory = $true)]
    [ValidateNotNullOrEmpty()]
    [string]$OnboardBridgeHost,

    [Parameter(Mandatory = $true)]
    [ValidateNotNullOrEmpty()]
    [string]$CalibrationId,

    [switch]$Force
)

$ErrorActionPreference = "Stop"
$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$projectRoot = Split-Path -Parent $scriptRoot
$hostConfig = Join-Path $scriptRoot "ground_station.local.ps1"
$artifactRoot = Join-Path $projectRoot "artifacts\deployment"
$onboardConfig = Join-Path $artifactRoot "vla_stack.env"

if ((Test-Path -LiteralPath $hostConfig) -and -not $Force) {
    throw "$hostConfig already exists; use -Force only when rotating both host and onboard tokens."
}

function New-RandomToken {
    $bytes = New-Object byte[] 32
    [System.Security.Cryptography.RandomNumberGenerator]::Create().GetBytes($bytes)
    return ([BitConverter]::ToString($bytes)).Replace("-", "").ToLowerInvariant()
}

$bridgeToken = New-RandomToken
$observationToken = New-RandomToken
$operatorToken = New-RandomToken
New-Item -ItemType Directory -Force -Path $artifactRoot | Out-Null

$hostLines = @(
    "`$env:HOST_ONBOARD_IP = `"$HostOnboardIp`"",
    "`$env:HOST_OPERATOR_IP = `"$HostOperatorIp`"",
    "`$env:ONBOARD_BRIDGE_HOST = `"$OnboardBridgeHost`"",
    '$env:ONBOARD_BRIDGE_PORT = "50051"',
    "`$env:ONBOARD_BRIDGE_TOKEN = `"$bridgeToken`"",
    "`$env:ONBOARD_OBSERVATION_TOKEN = `"$observationToken`"",
    "`$env:OPERATOR_CONTROL_TOKEN = `"$operatorToken`"",
    '$env:LIVE_CONTROL_CONFIRMATION = "I_ACCEPT_REAL_FLIGHT_CONTROL"',
    "`$env:EXPECTED_CALIBRATION_ID = `"$CalibrationId`"",
    '$env:EXPECTED_WORLD_FRAME = "world"',
    '$env:EXPECTED_BODY_FRAME = "base_link"',
    '$env:EXPECTED_CAMERA_FRAME = "vla_usb_camera_optical_frame"',
    '$env:OBSERVATION_K_FRAMES = "5"',
    '$env:VLA_OBSERVATION_MODE = "calibrated"',
    '$env:ONBOARD_MAX_CLOCK_OFFSET_MS = "300"',
    '$env:OBSERVATION_MAX_FUTURE_SKEW_MS = "300"',
    '$env:OPENVLA_URL = "http://127.0.0.1:5007"',
    '$env:PI05_HOST = "127.0.0.1"',
    '$env:PI05_PORT = "8000"',
    '$env:OPENVLA_PROJECT_ROOT = "REQUIRED"',
    '$env:OPENVLA_MODEL_PATH = "REQUIRED"',
    '$env:PI05_WSL_DISTRO = "Ubuntu-22.04"',
    '$env:PI05_WSL_USER = "REQUIRED"',
    '$env:PI05_OPENPI_ROOT = "REQUIRED"',
    '$env:PI05_CHECKPOINT_PATH = "REQUIRED"',
    '$env:PI05_POLICY_CONFIG = "pi05_uav_flow_lora"'
)
$hostLines | Set-Content -LiteralPath $hostConfig -Encoding UTF8

$onboardLines = @(
    'export ENABLE_FLIGHT_STACK=I_UNDERSTAND_THIS_STARTS_PX4CTRL',
    "export VLA_BACKEND_URL=http://${HostOnboardIp}:8080",
    "export VLA_BRIDGE_TOKEN='$bridgeToken'",
    "export VLA_OBSERVATION_TOKEN='$observationToken'",
    "export VLA_CALIBRATION_ID='$CalibrationId'",
    'export VLA_CALIBRATION_VALIDATED=I_VALIDATED_CAMERA_INFO_AND_TF',
    'export VLA_BRIDGE_MODE=preview',
    'export VLA_OBSERVATION_MODE=calibrated',
    'export VLA_ACTION_CHUNK_SAMPLE_COUNT=8',
    'export MULTIPOINT_START_PLAN=0',
    'export MULTIPOINT_BACK_PLAN=0',
    'export MULTIPOINT_AUTO_PLANNING=0',
    'export MULTIPOINT_AUTO_LANDING=0',
    "export VLA_HOST_IP=${HostOnboardIp}",
    'export VLA_WORLD_FRAME=world',
    'export VLA_BODY_FRAME=base_link',
    'export VLA_START_USB_CAMERA=1',
    'export VLA_USB_VIDEO_DEVICE=/dev/v4l/by-id/usb-KINGSEN_KS2A418-2.0-video-index0',
    'export VLA_USB_CAMERA_INFO_URL=file://${HOME}/.ros/camera_info/head_camera.yaml',
    'export VLA_USB_IMAGE_WIDTH=640',
    'export VLA_USB_IMAGE_HEIGHT=480',
    'export VLA_USB_FRAMERATE=30',
    'export VLA_USB_PIXEL_FORMAT=mjpeg',
    'export VLA_CAMERA_FRAME=vla_usb_camera_optical_frame',
    'export VLA_CAMERA_INFO_TOPIC=/vla_usb_camera/camera_info',
    'export VLA_IMAGE_COMPRESSED_TOPIC=/vla_usb_camera/image_raw/compressed',
    'export VLA_IMAGE_RAW_TOPIC=/vla_usb_camera/image_raw',
    'export VLA_IMAGE_TRANSPORT=compressed'
)
$ascii = New-Object System.Text.ASCIIEncoding
[System.IO.File]::WriteAllText(
    $onboardConfig,
    (($onboardLines -join "`n") + "`n"),
    $ascii
)

Write-Host "Created private host config: $hostConfig" -ForegroundColor Green
Write-Host "Created matching onboard config: $onboardConfig" -ForegroundColor Green
Write-Host "Token values were intentionally not printed."
