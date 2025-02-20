#!/bin/bash

# Ensure we are in the root of the repository
cd "$(git rev-parse --show-toplevel)"

# Initialize and update all submodules
git submodule update --init --recursive

# Read the .gitmodules file to get the path and branch for each submodule
git config -f .gitmodules --get-regexp '^submodule\..*\.path$' | while read -r key path
do
    # Extract the submodule name from the key
    name=$(echo "$key" | sed 's/^submodule\.\(.*\)\.path$/\1/')

    # Get the branch name for the submodule
    branch=$(git config -f .gitmodules --get "submodule.$name.branch")

    # Go into the submodule directory
    cd "$path"

    # Check out the specified branch
    git checkout "$branch"

    # Pull the latest changes from the specified branch
    git pull origin "$branch"

    # Go back to the root of the repository
    cd - > /dev/null
done

echo "Submodules have been updated to the latest commits on their respective branches."
