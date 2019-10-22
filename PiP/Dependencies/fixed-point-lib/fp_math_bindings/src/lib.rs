use fp_math::Fp64;

#[no_mangle]
pub extern fn fp64_add(a: Fp64, b: Fp64) -> Fp64 {
    a + b
}

#[no_mangle]
pub extern fn fp64_sub(a: Fp64, b: Fp64) -> Fp64 {
    a - b
}

#[no_mangle]
pub extern fn fp64_mul(a: Fp64, b: Fp64) -> Fp64 {
    a * b
}

#[no_mangle]
pub extern fn fp64_div(a: Fp64, b: Fp64) -> Fp64 {
    a / b
}

#[no_mangle]
pub extern fn fp64_mod(a: Fp64, b: Fp64) -> Fp64 {
    a % b
}

#[no_mangle]
pub extern fn fp64_sin(f: Fp64) -> Fp64 {
    f.sin()
}

#[no_mangle]
pub extern fn fp64_cos(f: Fp64) -> Fp64 {
    f.cos()
}

#[no_mangle]
pub extern fn fp64_sqrt(f: Fp64, iterations: u32) -> Fp64 {
    f.sqrt(iterations as usize)
}

#[no_mangle]
pub extern fn fp64_reciprocal_sqrt(f: Fp64, iterations: u32) -> Fp64 {
    f.reciprocal_sqrt(iterations as usize)
}

#[no_mangle]
pub extern fn fp64_half(f: Fp64) -> Fp64 {
    f.half()
}

#[no_mangle]
pub extern fn fp64_double(f: Fp64) -> Fp64 {
    f.double()
}

#[no_mangle]
pub extern fn fp64_pow(f: Fp64, exponent: u32) -> Fp64 {
    f.pow(exponent as usize)
}


#[no_mangle]
pub extern fn fp64_from_i32(i: i32) -> Fp64 {
    Fp64::from(i)
}

#[no_mangle]
pub extern fn fp64_to_i32(f: Fp64) -> i32 {
    f.into()
}

#[no_mangle]
pub extern fn fp64_from_i64(i: i64) -> Fp64 {
    Fp64::from(i)
}

#[no_mangle]
pub extern fn fp64_to_i64(f: Fp64) -> i64 {
    f.into()
}

#[no_mangle]
pub extern fn fp64_from_f32(f: f32) -> Fp64 {
    Fp64::from(f)
}

#[no_mangle]
pub extern fn fp64_to_f32(f: Fp64) -> f32 {
    f.into()
}

#[no_mangle]
pub extern fn fp64_from_f64(f: f64) -> Fp64 {
    Fp64::from(f)
}

#[no_mangle]
pub extern fn fp64_to_f64(f: Fp64) -> f64 {
    f.into()
}
