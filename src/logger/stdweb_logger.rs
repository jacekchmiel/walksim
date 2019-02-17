/// Really simple stdweb js console based backend for log crate

use log::{Log, Level, LevelFilter, Metadata, Record, set_boxed_logger};
use stdweb::{console, __internal_console_unsafe, js, _js_impl};

struct StdwebLogger {
    max_level: LevelFilter
}

impl Log for StdwebLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= self.max_level
    }

    fn log(&self, record: &Record) {
        let metadata = record.metadata();
        if self.enabled(metadata) {
            let level = metadata.level();
            let message = format!("[{}] {}", record.module_path().unwrap_or(""), record.args());
            match level {
                Level::Error => console!(error, message),
                _ => console!(log, message),
            }
        }
    }

    fn flush(&self) {

    }
}

pub fn init(max_level: LevelFilter) {
    let logger: Box<dyn Log> = Box::new(StdwebLogger {max_level});
    let r = set_boxed_logger(logger);

    if r.is_ok() {
        ::log::set_max_level(max_level);
    }
}