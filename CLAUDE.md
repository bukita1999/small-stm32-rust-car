# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an embedded Rust project template based on the Knurling Tools ecosystem for STM32 microcontroller development. It uses:
- `probe-rs` for flashing and debugging
- `defmt` for structured logging 
- `flip-link` for stack overflow protection
- `cortex-m-rt` for runtime initialization

## Key Configuration Requirements

Before building, you must configure:

1. **Target chip in `.cargo/config.toml`**: Replace `$CHIP` with your specific chip (see `probe-rs chip list`)
2. **Compilation target in `.cargo/config.toml`**: Uncomment the appropriate target for your Cortex-M variant
3. **HAL dependency in `Cargo.toml`**: Add your hardware abstraction layer (e.g., `stm32f4xx-hal`)
4. **HAL import in `src/lib.rs`**: Import the HAL crate for memory layout

## Build and Run Commands

- `cargo run` - Build and flash the default binary
- `cargo rb <binary_name>` - Run a specific binary (alias for `cargo run --bin`)
- `cargo rrb <binary_name>` - Run binary in release mode
- `cargo build` - Build without flashing
- `cargo build --release` - Optimized build

Available binaries in `src/bin/`:
- `hello` - Basic hello world
- `format` - Formatting examples
- `levels` - Log level examples  
- `bitfield` - Bitfield operations
- `panic` - Panic handling demo
- `overflow` - Stack overflow demo

## Testing

- `cargo test --lib` - Run unit tests in `src/lib.rs`
- `cargo test --test integration` - Run integration tests in `tests/integration.rs`
- `cargo test` - Run all tests (requires binary test = false in Cargo.toml)

## Architecture

- **Library crate** (`src/lib.rs`): Contains shared initialization, panic handlers, and utilities
- **Binary targets** (`src/bin/`): Individual examples and applications
- **Integration tests** (`tests/`): Hardware-in-the-loop testing with custom harness
- **Cargo configuration** (`.cargo/config.toml`): Target-specific build and runner settings

The project uses `#![no_std]` and `#![no_main]` for bare-metal embedded development. Memory layout and runtime are provided by the HAL crate and `cortex-m-rt`.

## Development Notes

- The template uses placeholder values (`{{authors}}`, `{{project-name}}`, `{{crate_name}}`) that should be replaced
- RTT buffer size can be adjusted via `DEFMT_RTT_BUFFER_SIZE` environment variable if running out of memory
- Custom log formats can be configured in the probe-rs runner command