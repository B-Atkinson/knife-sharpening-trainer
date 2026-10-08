## Rust terminology

Yes, calling it the “`mat3` module” is correct, with a small distinction between the module and its source file:

> `mat3.rs` is the source file backing the `mat3` module in the `sharpener-math` library crate.

The relevant terms are:

- **Workspace**: Collection of related Cargo packages represented by the root `Cargo.toml`.
- **Package**: A directory described by a `Cargo.toml`, such as `crates/sharpener-math`.
- **Crate**: A Rust compilation unit. Here, `sharpener-math` contains a library crate.
- **Module**: A namespace organizing Rust items. crates/sharpener-math/src/lib.rs:3 declares:

  ```
  pub mod mat3;
  ```

  Rust then uses `mat3.rs` as that module’s source.
- **Struct/type**: `Mat3` is a public struct type defined inside the `mat3` module.
- **Enum/type**: `Mat3Error` is a public enum type.
- **Inherent implementation**: `impl Mat3 { ... }` defines methods and associated functions belonging directly to `Mat3`.
- **Trait implementation**: `impl Mul<Vec3> for Mat3`, `impl AbsDiffEq for Mat3`, and similar blocks implement traits for the type.
- **Item**: General term encompassing structs, enums, functions, modules, traits, and implementation blocks.
- **Public API**: The publicly accessible types, methods, functions, and trait implementations exposed by the crate.

A natural way to describe your work would be:

> “I implemented the `Mat3` type and its tests in the `mat3` module of the `sharpener-math` crate.”

Or, when specifically discussing the physical code:

> “The implementation is in the `mat3.rs` source file.”