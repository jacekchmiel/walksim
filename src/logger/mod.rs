use cfg_if::cfg_if;

cfg_if! {
    if #[cfg(target_arch = "wasm32")] {
        mod stdweb_logger;
        fn init_impl() {
            stdweb_logger::init(log::LevelFilter::Debug);
        }
    } else {
        fn init_impl() {
            let env = env_logger::Env::default()
                .filter_or("MY_LOG_LEVEL", "trace")
                .write_style_or("MY_LOG_STYLE", "always");

            env_logger::init_from_env(env);
        }
    }

}

pub fn init() {
    init_impl();
}
