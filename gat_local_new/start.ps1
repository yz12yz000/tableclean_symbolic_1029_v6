# Start only the OpenCode client. No robot command is executed here.
$ErrorActionPreference = 'Stop'

if (-not (Get-Command opencode -ErrorAction SilentlyContinue)) {
    throw 'OpenCode was not found on PATH. Install/configure OpenCode before starting this project.'
}

if (-not (Test-Path -LiteralPath (Join-Path $PSScriptRoot '.git'))) {
    throw 'Project boundary is missing. Run git init in this project folder before starting OpenCode to avoid inheriting parent-project model settings.'
}

# Authentication and model selection are managed inside OpenCode.
# Deliberately do not read .env or require a local model server.
Push-Location $PSScriptRoot
try {
    & opencode
    $armClientExitCode = $LASTEXITCODE
} finally {
    Pop-Location
}
exit $armClientExitCode
