ARG UBUNTU=22.04
#ARG TYPE=base

# FROM wpilib/ubuntu-${TYPE}:${UBUNTU}

# # Create a user named OrangeDynamite
# RUN useradd -ms /bin/bash OrangeDynamite

# # Add OrangeDynamite to the sudo group
# RUN usermod -aG sudo OrangeDynamite

# # Set password for the user
# RUN echo "OrangeDynamite:2583" | chpasswd

# # Install toolchain
# RUN curl -SL https://github.com/wpilibsuite/opensdk/releases/download/v2024-1/cortexa9_vfpv3-roborio-academic-2024-x86_64-linux-gnu-Toolchain-12.1.0.tgz | sh -c 'mkdir -p /usr/local && cd /usr/local && tar xzf - --strip-components=2'

# USER OrangeDynamite
# WORKDIR /home/OrangeDynamite

# # Change the shell prompt
# RUN echo 'PS1="OrangeDynamite@FRC-2024:\w\$ "' >> ~/.bashrc

FROM mcr.microsoft.com/devcontainers/typescript-node:18-bookworm

RUN git config --system codespaces-theme.hide-status 1

RUN wget https://packages.wpilib.workers.dev/installer/v2024.3.1/Linux/WPILib_Linux-2024.3.1.tar.gz
RUN tar -xzvf WPILib_Linux-2024.3.1.tar.gz
RUN cd WPILib_Linux-2024.3.1


USER node
RUN npm install -g node-gyp
RUN YARN_CACHE="$(yarn cache dir)" && rm -rf "$YARN_CACHE" && ln -s /vscode-dev/yarn-cache "$YARN_CACHE"
RUN echo 'export DISPLAY="${DISPLAY:-:1}"' | tee -a ~/.bashrc >> ~/.zshrc

USER root
CMD chown node:node /vscode-dev && sudo -u node mkdir -p /vscode-dev/yarn-cache && sleep inf