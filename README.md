[![Crates.io](https://meritbadge.herokuapp.com/toxicblend)](https://crates.io/crates/toxicblend)
[![Documentation](https://docs.rs/toxicblend/badge.svg)](https://docs.rs/toxicblend)
[![Workflow](https://github.com/eadf/toxicblend.rs/workflows/Rust/badge.svg)](https://github.com/eadf/toxicblend.rs/workflows/Rust/badge.svg)
[![Workflow](https://github.com/eadf/toxicblend.rs/workflows/Clippy/badge.svg)](https://github.com/eadf/toxicblend.rs/workflows/Clippy/badge.svg)
[![dependency status](https://deps.rs/crate/toxicblend/0.0.1/status.svg)](https://deps.rs/crate/toxicblend/0.0.1)


# toxicblend.rs
Work in progress.\
This will be a rust port of my scala based project [toxicblend](https://github.com/toxicblend/toxicblend)

## Blender addon installation
Follow instructions in [install_as_blender_addon.md](blender_addon/install_as_blender_addon.md)

## Rust requirement

Requires `#![feature(hash_drain_filter)]` and `#![feature(map_first_last)]` i.e. `rust +nightly`

## Run local server
The blender addon is based on a client-server model using `grpc`.
The blender addon is the client and it only connects to `localhost`.
The server is local as well, run it with this command:
```
cargo run --bin server --release
```


