#!/bin/bash

# Print error msg for none argument
if [ "$1" = "" ]
then
    echo "Illegal argument"
    echo "Usage: "
    echo "$ sh change_pkg_name.sh <new_pkg_name>"
    exit
fi


# Get old pkg name from CMakeLists.txt 
old_pkg_name=$(cat CMakeLists.txt | grep "^project" | cut -d "(" -f2 | cut -d ")" -f1)

# Replace name at CMakeLists.txt
sed "s/$old_pkg_name/$1/g" CMakeLists.txt > CMakeLists.txt.tmp
rm CMakeLists.txt
mv CMakeLists.txt.tmp CMakeLists.txt

# Replace name at package.xml
sed "s/$old_pkg_name/$1/g" package.xml > package.xml.tmp
rm package.xml
mv package.xml.tmp package.xml

# Replace directory name
mv ../$old_pkg_name ../$1

echo "Successfully change package name from $old_pkg_name to $1"
echo "You should execute catkin build to recompile package"
