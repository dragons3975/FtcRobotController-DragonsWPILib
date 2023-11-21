ECHO OFF

cd %~dp0
git config core.symlinks true
git submodule init
git submodule update
git reset --hard
pause