#!/usr/bin/env bash

set -euo pipefail

export RUSTFLAGS="-D warnings"

locked=()
if [[ -n "${CI:-}" ]]; then
    locked=(--locked)
fi

failures=()

run() {
    local status=0
    if [[ -n "${GITHUB_ACTIONS:-}" ]]; then
        echo "::group::$*"
        "$@" || status=$?
        echo "::endgroup::"
    else
        echo "==> $*"
        "$@" || status=$?
    fi
    if ((status != 0)); then
        failures+=("$*")
        if [[ -n "${GITHUB_ACTIONS:-}" ]]; then
            echo "::error::failed: $*"
        fi
    fi
}

clippy() {
    local features=$1
    shift
    run cargo clippy "${locked[@]}" --all-targets --no-default-features --features="$features" "$@"
}

cargo_test() {
    local features=$1
    shift
    run cargo test "${locked[@]}" --no-fail-fast --no-default-features --features="$features" "$@"
}

group_core() {
    local features
    local sets=(
        "" # Default
        "multi-threaded"
        "gpu-tesselator"
    )
    for features in "${sets[@]}"; do
        clippy "$features"
    done
    for features in "${sets[@]}"; do
        cargo_test "$features"
    done
}

# Needs a C and a C++ compiler, and a GPU for the renderer cases (a software
# Vulkan one such as lavapipe works).
group_c() {
    local features
    local sets=(
        "c"
        "c,multi-threaded"
        "cpp"
        "c,cpp"
        "c,cpp,multi-threaded"
    )
    for features in "${sets[@]}"; do
        clippy "$features"
    done
    cargo_test "c" --lib bindings
    cargo_test "c" --test c_bindings
    cargo_test "c,multi-threaded" --test c_bindings
    # Alone, so the library has to export everything the C++ header declares
    cargo_test "cpp" --test cpp_bindings
}

# Needs python3 (or PYO3_PYTHON), and a GPU for the renderer cases (a
# software Vulkan one such as lavapipe works).
group_python() {
    local features
    local sets=(
        "python"
        "python,multi-threaded"
    )
    for features in "${sets[@]}"; do
        clippy "$features"
    done
    for features in "${sets[@]}"; do
        cargo_test "$features" --test python_bindings
    done
}

# Needs wasm-pack, the wasm32-unknown-unknown target, Node.js and the
# `webgpu` package (`npm ci` in tests/js), and a GPU for the renderer cases (a
# software Vulkan one such as lavapipe works). `multi-threaded` can not be
# built for JS.
group_js() {
    clippy "js"
    clippy "js" --target wasm32-unknown-unknown
    cargo_test "js" --test js_bindings
}

groups=("$@")
if [[ ${#groups[@]} -eq 0 ]]; then
    groups=(core c python js)
fi

for group in "${groups[@]}"; do
    case "$group" in
        core | c | python | js) "group_$group" ;;
        *)
            echo "unknown group: $group (expected core, c, python or js)" >&2
            exit 1
            ;;
    esac
done

if ((${#failures[@]} != 0)); then
    echo "failed steps:" >&2
    printf '  %s\n' "${failures[@]}" >&2
    exit 1
fi
