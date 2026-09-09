# Run manually in an elevated PowerShell, only on the new laptop.
# This does not enable control output, alter network profile, or contact the UAV.
$ErrorActionPreference = 'Stop'
$identity = [Security.Principal.WindowsIdentity]::GetCurrent()
$principal = New-Object Security.Principal.WindowsPrincipal($identity)
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    throw 'Run this script in an Administrator PowerShell after checking config.json.'
}
$config = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'config.json') -Raw | ConvertFrom-Json
$peer = $null
if (-not [Net.IPAddress]::TryParse([string]$config.Environment.ONBOARD_BRIDGE_HOST, [ref]$peer)) {
    throw 'ONBOARD_BRIDGE_HOST must be the onboard IP address.'
}
$port = 0
if (-not [int]::TryParse([string]$config.Port,[ref]$port) -or $port -lt 1024 -or $port -gt 65535) {
    throw 'Invalid Port.'
}
$name = "VLA-Portable-Onboard-$port"
$existing = Get-NetFirewallRule -Name $name -ErrorAction SilentlyContinue
if ($existing) { throw "Rule $name already exists. Inspect it manually; it was not changed." }
New-NetFirewallRule -Name $name -DisplayName $name -Direction Inbound -Action Allow -Protocol TCP -LocalPort $port -RemoteAddress $peer.IPAddressToString -Profile Private | Out-Null
Write-Host "Added Private-network-only TCP $port rule restricted to onboard IP $peer."
Write-Host "To remove this rule later: Remove-NetFirewallRule -Name $name"
