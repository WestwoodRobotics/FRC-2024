#!/bin/sh


echo "Post-create script starting..."

# Check if wget is installed
if ! command -v wget &> /dev/null
then
    echo "wget could not be found, installing..."
    sudo sudo apt update && sudo apt install -y wget
else
    sudo echo "wget is already installed"
fi

# Check if openjdk-21-jre is installed
if ! dpkg -l | grep -q openjdk-21-jre
then
    echo "openjdk-21-jre could not be found, installing..."
    sudo sudo apt update && sudo apt install -y openjdk-21-jre
else
    sudo echo "openjdk-21-jre is already installed"
fi

# Download the vscode-wpilib-2024.3.2.vsix file
sudo wget "https://github.com/wpilibsuite/vscode-wpilib/releases/download/v2024.3.2/vscode-wpilib-2024.3.2.vsix"

# Install the downloaded vsix file on VSCode
code --install-extension vscode-wpilib-2024.3.2.vsix

# Remove the downloaded vsix file after installation
sudo rm vscode-wpilib-2024.3.2.vsix

# Set ownership of /home/vscode directory to vscode user
sudo chown -R vscode:vscode /home/vscode

export JAVA_HOME=/usr/lib/jvm/java-21-openjdk-amd64
export PATH=$JAVA_HOME/bin:$PATH
source ~/.bashrc

echo "Post-create script finished."
