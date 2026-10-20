// The C++ header declares the same functions, so `cpp` alone has to export them too
#[cfg(any(feature = "c", feature = "cpp"))]
mod c;
