# Installation
Install Rust (https://rustup.rs) and add to PATH

Navigate to `fp_math/fp_math_bindings`.

From command line run the commands `cargo build` and `cargo build --release`.

`fp_math_bindings.dll`, `fp_math_bindings.dll.lib`, and `fp_math_bindings.lib` 
can be found in the target/debug and target/release directories.

`fp_math.h` is located in the include directory and should work for either C or 
C++. 


# Additional Notes
A `float` is `f32`, a `double` is `f64`. For `i32`, `i64`, and `u32`, 
use `int32_t`, `int64_t`, and `uint32_t` from stdint.h respectively.
