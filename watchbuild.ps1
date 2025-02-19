$watchPath = "E:\projectAnnihilation"
$filter = "*.cpp", "*.h", "CMakeLists.txt"
$lastWrite = @{}

# Detect file changes
$watcher = New-Object System.IO.FileSystemWatcher
$watcher.Path = $watchPath
$watcher.Filter = "*.*"
$watcher.IncludeSubdirectories = $true
$watcher.EnableRaisingEvents = $true

Write-Host "Watching for changes in $watchPath..."

while ($true) {
    Start-Sleep -Seconds 1  # Prevent instant loops

    $changedFiles = Get-ChildItem -Path $watchPath -Recurse -Include $filter | 
        Where-Object { $_.LastWriteTime -ne $lastWrite[$_.FullName] }

    if ($changedFiles) {
        Write-Host "Change detected! Rebuilding..."
        
        # Update timestamps to avoid infinite loop
        foreach ($file in $changedFiles) {
            $lastWrite[$file.FullName] = $file.LastWriteTime
        }

        # Run build command
        Start-Process -NoNewWindow -Wait -FilePath "cmake" -ArgumentList "--build build"
        
        Write-Host "Build complete. Watching for more changes..."
    }
}
