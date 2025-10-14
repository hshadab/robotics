# Multi-stage build for atlas_argmax_prover pinned to ICME-Lab/jolt-atlas
# Produces a runnable image and enables extracting the binary to host.

# Use fully-qualified image name for Podman compatibility
ARG RUST_IMAGE=docker.io/library/rust:1.90-bullseye
ARG JOLT_ATLAS_GIT=https://github.com/ICME-Lab/jolt-atlas
ARG JOLT_ATLAS_REF=main

FROM ${RUST_IMAGE} AS builder
ARG JOLT_ATLAS_GIT
ARG JOLT_ATLAS_REF

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates build-essential pkg-config clang cmake git \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /opt

# Clone JOLT Atlas (workspace with onnx-tracer + zkml-jolt-core)
RUN git clone ${JOLT_ATLAS_GIT} jolt-atlas \
 && cd jolt-atlas \
 && git checkout ${JOLT_ATLAS_REF}

# Copy our helper crate into jolt-atlas workspace
COPY atlas_argmax_prover /opt/jolt-atlas/atlas_argmax_prover

# Fix dependency paths in helper to reference local workspace members
RUN sed -i 's#\(zkml-jolt-core = { path = \)"\../jolt-atlas/zkml-jolt-core"#\1"../zkml-jolt-core"#' /opt/jolt-atlas/atlas_argmax_prover/Cargo.toml \
 && sed -i 's#\(onnx-tracer = { path = \)"\../jolt-atlas/onnx-tracer"#\1"../onnx-tracer"#' /opt/jolt-atlas/atlas_argmax_prover/Cargo.toml

# Add helper as a workspace member in jolt-atlas
RUN sed -i 's#members = \["onnx-tracer", "zkml-jolt-core"\]#members = ["onnx-tracer", "zkml-jolt-core", "atlas_argmax_prover"]#' /opt/jolt-atlas/Cargo.toml || true

WORKDIR /opt/jolt-atlas

# Vendor sxt-dory and align arkworks versions to a16z dev/twist-shout
RUN git clone https://github.com/spaceandtimefdn/sxt-dory vendor-dory \
 && sed -i 's/^ark-bn254 = \"0.5.0\"/ark-bn254 = { git = \"https:\/\/github.com\/a16z\/arkworks-algebra\", branch = \"dev\/twist-shout\" }/' vendor-dory/Cargo.toml \
 && sed -i 's/^ark-ec = { version = \"0.5.0\", default-features = false, features = \[\n    \"parallel\",\n\] }/ark-ec = { git = \"https:\/\/github.com\/a16z\/arkworks-algebra\", branch = \"dev\/twist-shout\", default-features = false, features = [\n    \"parallel\",\n] }/' vendor-dory/Cargo.toml \
 && sed -i 's/^ark-ff = { version = \"0.5.0\", default-features = false, features = \[\n    \"parallel\",\n\] }/ark-ff = { git = \"https:\/\/github.com\/a16z\/arkworks-algebra\", branch = \"dev\/twist-shout\", default-features = false, features = [\n    \"parallel\",\n] }/' vendor-dory/Cargo.toml \
 && printf '\n[patch."https://github.com/spaceandtimefdn/sxt-dory"]\ndory = { path = "vendor-dory" }\n' >> Cargo.toml

# Build the helper from inside jolt-atlas workspace (picks up patches and vendored dory)
RUN cargo build --release -p atlas_argmax_prover

FROM debian:bullseye-slim AS runner
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates \
  && rm -rf /var/lib/apt/lists/*
COPY --from=builder /opt/jolt-atlas/target/release/atlas_argmax_prover /usr/local/bin/atlas_argmax_prover
ENTRYPOINT ["/usr/local/bin/atlas_argmax_prover"]
