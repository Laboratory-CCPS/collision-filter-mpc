FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    git \
    cmake \
    python3-pip \
    wget \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

COPY acados_install.sh /tmp/acados_install.sh
RUN chmod +x /tmp/acados_install.sh && /tmp/acados_install.sh -p -e -v --qpoases --osqp

RUN TERA_RENDERER_VERSION='0.2.0' && \
    _TERA_RENDERER_GITHUB_RELEASES="https://github.com/acados/tera_renderer/releases/download/v${TERA_RENDERER_VERSION}/" && \
    TERA_RENDERER_URL="${_TERA_RENDERER_GITHUB_RELEASES}/t_renderer-v${TERA_RENDERER_VERSION}-linux-amd64" && \
    mkdir -p /root/acados/bin && \
    wget -O /root/acados/bin/t_renderer "${TERA_RENDERER_URL}" && \
    chmod +x /root/acados/bin/t_renderer

RUN rm /tmp/acados_install.sh