@echo off
setlocal

set ROOT_DIR=%~dp0..
set PYTHON_BIN=python

echo Training nav priors...
echo   nav bin: %ROOT_DIR%\assets\navmesh_polygons.nav
echo   output : %ROOT_DIR%\assets\nav_priors.json

%PYTHON_BIN% "%ROOT_DIR%\tools\train_nav_priors.py" ^
  --nav-bin "%ROOT_DIR%\assets\navmesh_polygons.nav" ^
  --out "%ROOT_DIR%\assets\nav_priors.json" ^
  --k-paths 10 ^
  --turn-weight 110 ^
  --narrow-weight 140 ^
  --portal-hop-weight 6 ^
  --bonus-scale 45 ^
  %*

echo.
if errorlevel 1 (
  echo Training failed.
) else (
  echo Done.
)

pause
endlocal