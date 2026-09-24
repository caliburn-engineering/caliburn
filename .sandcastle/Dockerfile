FROM node:22-bookworm

# --- System dependencies ---
# git/curl/jq: baseline agent tooling. build-essential/cmake/ninja/pkg-config:
# reference/ is Eigen-based C++17 built with CMake FetchContent (needs git + TLS).
# xorg-dev + mesa headers: the scaffold-sim-viewer / scaffold-bode-plot skills
# generate GLFW+OpenGL+ImGui targets. These let `cmake --build` succeed headlessly;
# the viewers still cannot be *displayed* in the container — see AGENTS.md.
RUN apt-get update && apt-get install -y \
  git \
  curl \
  jq \
  ca-certificates \
  build-essential \
  cmake \
  ninja-build \
  pkg-config \
  xorg-dev \
  libgl1-mesa-dev \
  libglu1-mesa-dev \
  && rm -rf /var/lib/apt/lists/*

# --- GitHub CLI ---
RUN curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg \
  | dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" \
  | tee /etc/apt/sources.list.d/github-cli.list > /dev/null \
  && apt-get update && apt-get install -y gh \
  && rm -rf /var/lib/apt/lists/*

# Build-args for UID/GID alignment: sandcastle docker build-image
# defaults these to the host user's UID/GID so image-built files
# and bind-mounted files share an owner without runtime chown.
ARG AGENT_UID=1000
ARG AGENT_GID=1000

# Rename the base image's "node" user to "agent" and align UID/GID.
RUN groupmod -o -g $AGENT_GID node && usermod -o -u $AGENT_UID -g $AGENT_GID -d /home/agent -m -l agent node

# --- Coding agents ---
# Both harnesses are installed so main.ts can switch between pi() and claudeCode()
# without a rebuild.
#
# NOTE: sandcastle's stock pi template installs @mariozechner/pi-coding-agent,
# which is the OLD package name (stale at 0.73.x). pi has moved to
# @earendil-works/pi-coding-agent — this must match the host pi version.
RUN npm install -g \
  @earendil-works/pi-coding-agent \
  @anthropic-ai/claude-code

USER ${AGENT_UID}:${AGENT_GID}

WORKDIR /home/agent

# In worktree sandbox mode, Sandcastle bind-mounts the git worktree at /home/agent/workspace
# and overrides the working directory to /home/agent/workspace at container start.
# Structure your Dockerfile so that /home/agent/workspace can serve as the project root.
ENTRYPOINT ["sleep", "infinity"]
