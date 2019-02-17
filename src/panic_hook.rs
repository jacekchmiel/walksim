/// For reasons yet unknown. Cargo-web/stdweb is not always providing panic
/// stack traces in js console. (At the time of writing, panic in main() was
/// properly logged but panic in kiss3d crate was not. This hook attempts to fix
/// this issue.
///
/// This module is based on
/// [`console_error_panic_hook`](https://crates.io/crates/console_error_panic_hook)
/// crate.
use cfg_if::cfg_if;
use std::panic;

cfg_if! {
    if #[cfg(target_arch = "wasm32")] {

        use stdweb::{console, __internal_console_unsafe, js, _js_impl};

        fn hook_impl(info: &panic::PanicInfo) {
            console!(error, info.to_string());
        }

    } else {
        use std::io::{self, Write};

        fn hook_impl(info: &panic::PanicInfo) {
            let _ = writeln!(io::stderr(), "{}", info);
        }
    }
}

/// A panic hook for use with
/// [`std::panic::set_hook`](https://doc.rust-lang.org/nightly/std/panic/fn.set_hook.html)
/// that logs panics into
/// [`console.error`](https://developer.mozilla.org/en-US/docs/Web/API/Console/error).
///
/// On non-wasm targets, prints the panic to `stderr`.
pub fn hook(info: &panic::PanicInfo) {
    hook_impl(info);
}

/// Set the `console.error` panic hook the first time this is called. Subsequent
/// invocations do nothing.
#[inline]
pub fn set_once() {
    use std::sync::{Once, ONCE_INIT};
    static SET_HOOK: Once = ONCE_INIT;
    SET_HOOK.call_once(|| {
        panic::set_hook(Box::new(hook));
    });
}
