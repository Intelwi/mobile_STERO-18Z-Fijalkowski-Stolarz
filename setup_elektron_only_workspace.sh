#!/usr/bin/env bash

#compile elektron workspaces in a fakeroot environment
branch_name="melodic-setup-working"
if [ `git branch --list $branch_name` ]
then
    git clone https://github.com/RCPRG-ros-pkg/RCPRG_rosinstall
else
    git clone --single-branch -b $branch_name https://github.com/RCPRG-ros-pkg/RCPRG_rosinstall
fi
cd RCPRG_rosinstall
chmod +x setup.sh
./setup.sh -e -F -d build -b Release -i /opt

# coppy compiled elektron workspace to your /opt directory
sudo cp -r build/opt/ws_elektron /opt/
# remove workspace build folder 
cd ..
read -p "Would you like to remove the build directory? " -n 1 -r
echo    # (optional) move to a new line
if [[ $REPLY =~ ^[Yy]$ ]]
then
    rm -rf RCPRG_rosinstall
fi
