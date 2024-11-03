FROM mcr.microsoft.com/devcontainers/cpp:1-ubuntu-22.04

ARG REINSTALL_CMAKE_VERSION_FROM_SOURCE="3.22.2"

COPY .devcontainer/reinstall-cmake.sh /tmp/

RUN if [ "${REINSTALL_CMAKE_VERSION_FROM_SOURCE}" != "none" ]; then \
    chmod +x /tmp/reinstall-cmake.sh && /tmp/reinstall-cmake.sh ${REINSTALL_CMAKE_VERSION_FROM_SOURCE}; \
    fi \
    && rm -f /tmp/reinstall-cmake.sh

WORKDIR /tmp

# install all required packages
RUN apt update && export DEBIAN_FRONTEND=noninteractive && \
    apt -y install --no-install-recommends wget figlet clang-format cppcheck catch2 linux-tools-virtual hwdata

# cleanup tmp
RUN rm -rf /tmp/*

# install pyocd
RUN apt -y install python3 python3-pip netcat && \
    python3 -mpip install -U pyocd && \
    wget https://www.python.org/ftp/python/3.8.12/Python-3.8.12.tgz && \
    tar xzf Python-3.8.12.tgz && \
    cd Python-3.8.12 && \
    ./configure --enable-optimizations && \
    make altinstall

# install arm toolchain
RUN mkdir -p /opt/gcc-arm/ && \
    wget https://developer.arm.com/-/media/Files/downloads/gnu/12.3.rel1/binrel/arm-gnu-toolchain-12.3.rel1-x86_64-arm-none-eabi.tar.xz && \
    tar xvf arm-gnu-toolchain-12.3.rel1-x86_64-arm-none-eabi.tar.xz -C /opt/gcc-arm/ && \
    cd /opt/gcc-arm/ && \
    chown -R root:root arm-gnu-toolchain-12.3.rel1-x86_64-arm-none-eabi/

# install ncurses dependency
RUN apt -y install libncurses5 && ln -s /usr/lib/x86_64-linux-gnu/libncursesw.so.6 /usr/lib/x86_64-linux-gnu/libncursesw.so.5

# configure welcome splash screen
RUN cd /usr/share/figlet && wget https://raw.githubusercontent.com/xero/figlet-fonts/master/3d.flf && \
    echo "figlet -f 3d.flf Puya" >>/etc/bash.bashrc && \
    echo "echo Welcome to the Puya Playground development container!" >>/etc/bash.bashrc

# add a git lgb alias
RUN git config --global alias.lgb "log --graph --pretty=format:'%Cred%h%Creset -%C(yellow)%d%Creset %s %Cgreen(%cr) %C(bold blue)<%an>%Creset%n' --abbrev-commit --date=relative --branches"

# pyocd gdbserver -u remote:172.20.206.105:5555 --config Misc/pyocd.yaml --target py32f003x8 
# https://stackoverflow.com/questions/42938992/usbip-in-docker