use std::time::Duration;

use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    hal::modem::Modem,
    nvs::EspDefaultNvsPartition,
    sys::{EspError, ESP_ERR_TIMEOUT},
    timer::EspTaskTimerService,
    wifi::{AsyncWifi, AuthMethod, ClientConfiguration, Configuration, EspWifi},
};
use log::info;

const SSID: &str = "Stockholm Makerspace";
const PASSWORD: &str = "jagvetinte";
const WOKWI_SSID: &str = "Wokwi-GUEST";

pub async fn start_wifi(
    modem: Modem,
    sys_loop: EspSystemEventLoop,
    nvs: EspDefaultNvsPartition,
    timer_service: EspTaskTimerService,
    is_wokwi_simulator: bool,
) -> [u8; 6] {
    let wifi = AsyncWifi::wrap(
        EspWifi::new(modem, sys_loop.clone(), Some(nvs)).unwrap(),
        sys_loop,
        timer_service,
    )
    .unwrap();

    let mac = wifi.wifi().ap_netif().get_mac().unwrap();

    let mut wifi_loop = WifiLoop { wifi };
    wifi_loop.configure(is_wokwi_simulator).await.unwrap();
    wifi_loop.initial_connect().await.unwrap();

    tokio::spawn(async move {
        wifi_loop.stay_connected().await.unwrap();
    });

    mac
}

pub struct WifiLoop<'a> {
    pub wifi: AsyncWifi<EspWifi<'a>>,
}

impl<'a> WifiLoop<'a> {
    pub async fn configure(&mut self, is_wokwi_simulator: bool) -> Result<(), EspError> {
        let wifi_configuration = if is_wokwi_simulator {
            ClientConfiguration {
                ssid: WOKWI_SSID.try_into().unwrap(),
                bssid: None,
                auth_method: AuthMethod::None,
                password: "".try_into().unwrap(),
                channel: Some(6),
                ..Default::default()
            }
        } else {
            ClientConfiguration {
                ssid: SSID.try_into().unwrap(),
                bssid: None,
                auth_method: AuthMethod::WPA2Personal,
                password: PASSWORD.try_into().unwrap(),
                channel: None,
                ..Default::default()
            }
        };
        log::info!("Connecting to {}...", wifi_configuration.ssid);

        self.wifi
            .set_configuration(&Configuration::Client(wifi_configuration))?;

        info!("Starting Wi-Fi driver...");
        self.wifi.start().await
    }

    pub async fn initial_connect(&mut self) -> Result<(), EspError> {
        self.do_connect_loop(true).await
    }

    pub async fn stay_connected(mut self) -> Result<(), EspError> {
        self.do_connect_loop(false).await
    }

    async fn try_connect(wifi: &mut AsyncWifi<EspWifi<'a>>) -> Result<(), EspError> {
        wifi.connect().await?;

        info!("Waiting for association...");
        wifi.ip_wait_while(
            |wifi| wifi.is_up().map(|s| !s),
            Some(Duration::from_millis(5000)),
        )
        .await?;

        Ok(())
    }

    async fn do_connect_loop(&mut self, exit_after_first_connect: bool) -> Result<(), EspError> {
        let wifi = &mut self.wifi;
        loop {
            // Wait for disconnect before trying to connect again.  This loop ensures
            // we stay connected and is commonly missing from trivial examples as it's
            // way too difficult to showcase the core logic of an example and have
            // a proper Wi-Fi event loop without a robust async runtime.  Fortunately, we can do it
            // now!
            wifi.wifi_wait(|wifi| wifi.is_up(), None).await?;

            info!("Connecting to Wi-Fi...");
            if let Err(e) = Self::try_connect(wifi).await {
                match e.code() {
                    ESP_ERR_TIMEOUT => {
                        log::error!(
                            "Timeout when connecting to wifi. Trying again in a few seconds..."
                        );
                    }
                    _ => {
                        log::error!("Other error when connecting to wifi. Trying again in a few seconds: {e}");
                    }
                }

                // Sleep for a few seconds
                tokio::time::sleep(Duration::from_millis(2000)).await;
                continue;
            }

            if exit_after_first_connect {
                return Ok(());
            }
        }
    }
}
