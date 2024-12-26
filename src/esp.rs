use esp_idf_svc::sys::esp;

pub fn init_esp() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    // eventfd is needed by our mio poll implementation.  Note you should set max_fds
    // higher if you have other code that may need eventfd.
    let config = esp_idf_svc::sys::esp_vfs_eventfd_config_t { max_fds: 1 };

    esp! { unsafe { esp_idf_svc::sys::esp_vfs_eventfd_register(&config) } }.unwrap();
}
