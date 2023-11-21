ECHO OFF

git config core.symlinks true
git reset --hard
git submodule init
git submodule update
