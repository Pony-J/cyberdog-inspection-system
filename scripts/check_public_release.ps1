[CmdletBinding()]
param(
    [string]$Root = '',
    [int64]$MaxFileSizeMb = 20,
    [int64]$DemoGifMaxFileSizeMb = 60
)

$ErrorActionPreference = 'Stop'
if ([string]::IsNullOrWhiteSpace($Root)) {
    $scriptDirectory = Split-Path -Parent $MyInvocation.MyCommand.Path
    $Root = Split-Path -Parent $scriptDirectory
}
$resolvedRoot = (Resolve-Path -LiteralPath $Root).Path
$maxBytes = $MaxFileSizeMb * 1MB
$demoGifMaxBytes = $DemoGifMaxFileSizeMb * 1MB
$errors = [System.Collections.Generic.List[string]]::new()

$blockedExtensions = @(
    '.engine', '.plan', '.onnx', '.pt', '.pth', '.wts',
    '.pdmodel', '.pdiparams', '.so', '.a', '.bag', '.db3', '.mcap'
)

$skipDirectoryNames = @('.git', 'build', 'install', 'log', '__pycache__')
$files = Get-ChildItem -LiteralPath $resolvedRoot -Recurse -Force -File | Where-Object {
    $relative = $_.FullName.Substring($resolvedRoot.Length).TrimStart('\', '/')
    -not ($skipDirectoryNames | Where-Object { $relative -match "(^|[\\/])$([regex]::Escape($_))([\\/]|$)" })
}

foreach ($file in $files) {
    $relative = $file.FullName.Substring($resolvedRoot.Length).TrimStart('\', '/')
    $extension = $file.Extension.ToLowerInvariant()

    if ($blockedExtensions -contains $extension) {
        $errors.Add("blocked artifact: $relative")
    }

    $isDemoGif = $extension -eq '.gif' -and $relative -match '^media[\\/]demos[\\/]'
    $fileMaxBytes = if ($isDemoGif) { $demoGifMaxBytes } else { $maxBytes }
    if ($file.Length -gt $fileMaxBytes) {
        $sizeMb = [math]::Round($file.Length / 1MB, 2)
        $errors.Add("large file (${sizeMb} MB): $relative")
    }

    if ($extension -in @('.png', '.jpg', '.jpeg', '.gif', '.svg', '.ico')) {
        continue
    }

    try {
        $content = Get-Content -LiteralPath $file.FullName -Raw -ErrorAction Stop
    }
    catch {
        continue
    }

    $checks = @(
        @{ Name = 'private key'; Pattern = '-----BEGIN (RSA |EC |OPENSSH )?PRIVATE KEY-----' },
        @{ Name = 'credential in RTSP URL'; Pattern = 'rtsp://[^\s/:]+:[^\s/@]+@' },
        @{ Name = 'GitHub token'; Pattern = 'gh[pousr]_[A-Za-z0-9_]{20,}' },
        @{ Name = 'AWS access key'; Pattern = 'AKIA[0-9A-Z]{16}' },
        @{ Name = 'probable secret assignment'; Pattern = '(?im)^\s*(password|passwd|secret|api[_-]?key|access[_-]?token)\s*[:=]\s*["'']?(?!CHANGEME|EXAMPLE|YOUR_|\$\{|<)[^\s#"'']{8,}' }
    )

    foreach ($check in $checks) {
        if ($content -match $check.Pattern) {
            $errors.Add("$($check.Name): $relative")
        }
    }
}

if ($errors.Count -gt 0) {
    Write-Host 'Public release check failed:' -ForegroundColor Red
    $errors | Sort-Object -Unique | ForEach-Object { Write-Host "  - $_" -ForegroundColor Red }
    exit 1
}

Write-Host "Public release check passed: $($files.Count) files checked." -ForegroundColor Green
