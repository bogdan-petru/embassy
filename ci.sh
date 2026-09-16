#!/bin/bash

set -eo pipefail

if ! command -v cargo-batch &> /dev/null; then
    echo "cargo-batch could not be found. Install it with the following command:"
    echo ""
    echo "    cargo install --git https://github.com/embassy-rs/cargo-batch cargo --bin cargo-batch --locked"
    echo ""
    exit 1
fi

if ! command -v cargo-embassy-devtool &> /dev/null; then
    echo "cargo-embassy-devtool could not be found. Install it with the following command:"
    echo ""
    echo "    cargo install --git https://github.com/embassy-rs/cargo-embassy-devtool --locked"
    echo ""
    exit 1
fi

export RUSTFLAGS=-Dwarnings
export DEFMT_LOG=trace,embassy_hal_internal=debug,embassy_net_esp_hosted=debug,cyw43=info,cyw43_pio=info,smoltcp=info
if [[ -z "${CARGO_TARGET_DIR}" ]]; then
    export CARGO_TARGET_DIR=target_ci
fi

# always run check to prime cache
cargo embassy-devtool check --force-incremental

# Generated-map drift guard: the LPI2C Tock register map's offsets are
# generated from the PAC; a PAC bump without regeneration compiles fine
# and would otherwise only fail at hardware init (check_layout panic).
# After the cache-priming check, so the lockfile the resolver locks
# against exists on a fresh checkout. (On Windows, `python3` may be the
# Store stub while real Python is `python` — accept either.)
if command -v python3 &> /dev/null && python3 -c '' &> /dev/null; then
    mcxa_python=python3
elif command -v python &> /dev/null && python -c '' &> /dev/null; then
    mcxa_python=python
else
    echo "python not found: the generated-map drift check (embassy-mcxa/tools/"
    echo "gen_lpi2c_regs.py --check) cannot run. Install python3 — a guard that"
    echo "can be silently skipped is no guard."
    exit 1
fi

"$mcxa_python" embassy-mcxa/tools/gen_lpi2c_regs.py --check
# The one-shot interfaces now expand through Abacus at Rust compile time.
# Compare their declarations with the independent contract, then test the actual
# macro and its executable/rejection fixtures. HAL tests below check PAC layout.
cargo test --locked --manifest-path embassy-mcxa/tools/driver-contracts/Cargo.toml --test abacus_surface
cargo test --locked --manifest-path embassy-mcxa/experimental/abacus-registers/abacus-registers-dsl/Cargo.toml --lib
cargo test --locked --manifest-path embassy-mcxa/experimental/abacus-registers/Cargo.toml

# custom-mmio may be enabled in host-test/dev graphs, but must never leak into
# firmware. Test the checker as well as the real normal/build dependency graph.
# Match the MCXA5xx one-shot firmware manifest, target and default features.
# This policy check does not replace the separately linked firmware validation.
"$mcxa_python" -m unittest discover -s embassy-mcxa/tools -p check_one_shot_firmware_features_test.py
"$mcxa_python" embassy-mcxa/tools/check_one_shot_firmware_features.py \
    --manifest-path examples/mcxa5xx/Cargo.toml --features i2c-one-shot \
    --target thumbv8m.main-none-eabihf
"$mcxa_python" embassy-mcxa/tools/check_one_shot_firmware_features.py \
    --manifest-path examples/mcxa5xx/Cargo.toml --features uart-one-shot \
    --target thumbv8m.main-none-eabihf
# Exercise the MCXA pure decision logic and explicit DMA binding contracts on
# the host. The positive doctest must compile; missing/wrong-channel/erased
# binding examples must not. Keep this before the optional hardware-test gate.
mcxa_host_target=$(rustc -vV | sed -n 's/^host: //p')
for mcxa_chip in mcxa2xx mcxa5xx; do
    cargo test --locked --manifest-path embassy-mcxa/Cargo.toml \
        --target "$mcxa_host_target" --no-default-features --features "test,$mcxa_chip" --lib
    cargo test --locked --manifest-path embassy-mcxa/Cargo.toml \
        --target "$mcxa_host_target" --no-default-features --features "$mcxa_chip,one-shot-host-tests,uart-one-shot" --lib
    # Public negatives stay OFF custom-mmio; their executable must not acquire
    # the host recorder's linker-provider dependency.
    cargo test --locked --manifest-path embassy-mcxa/Cargo.toml \
        --target "$mcxa_host_target" --no-default-features --features "$mcxa_chip,test,i2c-one-shot" \
        --doc -- i2c::one_shot
    cargo test --locked --manifest-path embassy-mcxa/Cargo.toml \
        --target "$mcxa_host_target" --no-default-features --features "test,$mcxa_chip" \
        --doc -- "dma::DmaChannel<'a>::new"
done

# Both protocol and vocabulary libraries forbid unsafe code. Only the small
# layer-2 interface declares the unsafe Device contract; none has a normal
# PAC/HAL/timer dependency. Source/graph policy and mocks complement the HAL recorder.
mcxa_interface=embassy-mcxa/experimental/i2c-one-shot-interface
mcxa_vocabulary=embassy-mcxa/experimental/i2c-one-shot-vocabulary
mcxa_protocol=embassy-mcxa/experimental/i2c-one-shot-core
cargo generate-lockfile --manifest-path "$mcxa_interface/Cargo.toml"
cargo check --locked --manifest-path "$mcxa_interface/Cargo.toml" --target "$mcxa_host_target"
cargo check --locked --manifest-path "$mcxa_interface/Cargo.toml" --target "$mcxa_host_target" --features defmt
cargo generate-lockfile --manifest-path "$mcxa_vocabulary/Cargo.toml"
cargo test --locked --manifest-path "$mcxa_vocabulary/Cargo.toml" --target "$mcxa_host_target"
# As for the core below, defmt is type-checked without a host logger provider.
cargo check --locked --manifest-path "$mcxa_vocabulary/Cargo.toml" --target "$mcxa_host_target" --features defmt --tests
cargo generate-lockfile --manifest-path "$mcxa_protocol/Cargo.toml"
# Includes the independent generated-sequence properties (host dev-dependency
# only). The separate cargo-mutants runner is an explicit test-quality audit.
cargo test --locked --manifest-path "$mcxa_protocol/Cargo.toml" --target "$mcxa_host_target"
# defmt's target logger/linker symbols are supplied by firmware, not the host
# decision fixture. Type-check that feature here; linked firmware tests cover it.
cargo check --locked --manifest-path "$mcxa_protocol/Cargo.toml" --target "$mcxa_host_target" --features defmt --tests

# Execute the same L3 property helpers on finite native cases. This is NOT a
# symbolic proof: the separate run-kani.ps1 runner verifies all 3 L4 + 9 L3 harnesses.
cargo test --locked --manifest-path embassy-mcxa/experimental/i2c-one-shot-proofs/Cargo.toml \
    --target "$mcxa_host_target" --bin vocabulary

# Second IP: same contract schema/mechanical generator, independently reviewed
# UART hooks. Locks for these standalone libraries are generated before the
# host-only contract tool checks their normal dependency/source inventories.
for mcxa_layer in interface vocabulary core; do
    mcxa_uart="embassy-mcxa/experimental/uart-one-shot-$mcxa_layer/Cargo.toml"
    cargo generate-lockfile --manifest-path "$mcxa_uart"
    cargo test --locked --manifest-path "$mcxa_uart" --target "$mcxa_host_target"
    cargo check --locked --manifest-path "$mcxa_uart" --target "$mcxa_host_target" --features defmt --tests
done
mcxa_contracts=embassy-mcxa/tools/driver-contracts/Cargo.toml
# Board-free example bodies and independent bad/short/extra-data controls.
cargo generate-lockfile --manifest-path examples/nxp-shared/Cargo.toml
cargo test --locked --manifest-path examples/nxp-shared/Cargo.toml --target "$mcxa_host_target"
cargo check --locked --manifest-path examples/nxp-shared/Cargo.toml --target thumbv8m.main-none-eabihf
cargo test --locked --manifest-path "$mcxa_contracts" --target "$mcxa_host_target"
for mcxa_ip in i2c uart; do
    cargo run --locked --manifest-path "$mcxa_contracts" --target "$mcxa_host_target" -- check "$mcxa_ip" .
done
for mcxa_chip in mcxa2xx mcxa5xx; do
    # Plain UART MMIO fixture is a different link unit from the fail-closed
    # I2C custom-MMIO recorder; both suites must execute, not just type-check.
    cargo test --locked --manifest-path embassy-mcxa/Cargo.toml \
        --target "$mcxa_host_target" --no-default-features --features "$mcxa_chip,test,uart-one-shot" --lib
    for mcxa_uart_features in "$mcxa_chip,uart-one-shot" "$mcxa_chip,uart-one-shot,defmt"; do
        cargo check --locked --manifest-path embassy-mcxa/Cargo.toml \
            --target thumbv8m.main-none-eabihf --no-default-features --features "$mcxa_uart_features"
    done
    for mcxa_i2c_features in "$mcxa_chip,i2c-one-shot" "$mcxa_chip,i2c-one-shot,defmt"; do
        cargo check --locked --manifest-path embassy-mcxa/Cargo.toml \
            --target thumbv8m.main-none-eabihf --no-default-features --features "$mcxa_i2c_features"
    done
done
# Explicit linker configuration: RUSTFLAGS replaces, rather than appends to,
# the example's target rustflags. A check-only/empty linked ELF is insufficient.
(
    cd examples/mcxa5xx
    for mcxa_port_features in uart-one-shot uart-port-clock-div2; do
        RUSTFLAGS="$RUSTFLAGS -C linker=flip-link -C link-arg=-Tlink.x -C link-arg=-Tdefmt.x -C link-arg=--nmagic" \
            cargo build --locked --release --target thumbv8m.main-none-eabihf \
            --features "$mcxa_port_features" --bin uart-one-shot-sender --bin uart-one-shot-receiver
    done
    RUSTFLAGS="$RUSTFLAGS -C linker=flip-link -C link-arg=-Tlink.x -C link-arg=-Tdefmt.x -C link-arg=--nmagic" \
        cargo build --locked --release --target thumbv8m.main-none-eabihf \
            --features i2c-one-shot --bin i2c-one-shot-controller --bin i2c-one-shot-target \
            --bin i2c-one-shot-write-read-controller --bin i2c-one-shot-write-read-target
)
# Compile-only negative control: safe PAC writes/pops still compile under
# forbid(unsafe_code). The wake-path AST gate rejects them for ownership policy.
# Never link or run this fixture on a board.
mcxa_pac_control="$mcxa_protocol/tests/fixtures/pac-bypass-control/Cargo.toml"
cargo generate-lockfile --manifest-path "$mcxa_pac_control"
cargo check --locked --manifest-path "$mcxa_pac_control" --target thumbv8m.main-none-eabihf

# A source policy alone cannot detect lint-cap flag drift. Check actual compiler
# rejections and real adapter assertions on this exact committed revision, in
# isolated worktrees; unrelated compile/link failures do not count as a pass.
"$mcxa_python" -m unittest discover -s examples/mcxa5xx/one-shot/mutations/tests -v
"$mcxa_python" examples/mcxa5xx/one-shot/mutations/runner.py --baseline "$(git rev-parse HEAD)"

if [[ -z "${TELEPROBE_TOKEN-}" ]]; then
    echo No teleprobe token found, skipping running HIL tests
    exit
fi

cargo embassy-devtool build

# temporarily disabled, these boards are dead.
rm -rf out/tests/stm32f103c8
rm -rf out/tests/nrf52840-dk
rm -rf out/tests/nrf52833-dk
rm -rf out/tests/nrf5340-dk
rm -rf out/tests/nrf51422-dk
 
# disabled because these boards are not on the shelf
rm -rf out/tests/mspm0g3507

# rm out/tests/stm32wb55rg/wpan_mac
# rm out/tests/stm32wb55rg/wpan_ble

# unstable, I think it's running out of RAM?
rm out/tests/stm32f207zg/eth

# temporarily disabled, flaky.
rm out/tests/stm32f207zg/usart_rx_ringbuffered
rm out/tests/stm32l152re/usart_rx_ringbuffered

# doesn't work, gives "noise error", no idea why. usart_dma does pass.
rm out/tests/stm32u5a5zj/usart

# probe-rs error: "multi-core ram flash start not implemented yet"
# As of 2025-02-17 these tests work when run from flash
rm out/tests/pimoroni-pico-plus-2/multicore
rm out/tests/pimoroni-pico-plus-2/gpio_multicore
rm out/tests/pimoroni-pico-plus-2/spinlock_mutex_multicore
# Doesn't work when run from ram on the 2350
rm out/tests/pimoroni-pico-plus-2/flash
# This test passes locally but fails on the HIL, no idea why
rm out/tests/pimoroni-pico-plus-2/i2c
# The pico2 plus doesn't have the adcs hooked up like the picoW does.
rm out/tests/pimoroni-pico-plus-2/adc
# temporarily disabled
rm out/tests/pimoroni-pico-plus-2/pwm
rm out/tests/frdm-mcx-a266/trng

# flaky
rm out/tests/rpi-pico/pwm
rm out/tests/rpi-pico/cyw43-perf
rm out/tests/rpi-pico/uart_buffered
rm out/tests/rpi-pico/spi_async

rm out/tests/stm32h563zi/usart_dma

# tests are implemented but the HIL test farm doesn't actually have these boards, yet
rm -rf out/tests/stm32c071rb
rm -rf out/tests/stm32f100rd
rm -rf out/tests/stm32f107vc

teleprobe client run -r out/tests
