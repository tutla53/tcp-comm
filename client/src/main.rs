#![no_std]
#![no_main]

use{
    core::{
        net::Ipv4Addr,
        str::FromStr,
    },
    embassy_executor::Spawner,
    embassy_net::{tcp::TcpSocket, Runner, StackResources, Config, DhcpConfig},
    embassy_time::{Duration, Timer, Instant, with_timeout},
    esp_hal::{clock::CpuClock, rng::Rng, timer::timg::TimerGroup},
    esp_println::println,
    esp_wifi::{
        init,
        wifi::{
            ClientConfiguration,
            Configuration,
            WifiController,
            WifiDevice,
            WifiEvent,
            WifiStaDevice,
            WifiState,
        },
        EspWifiController,
    },
    embedded_io_async::Write,
    esp_alloc as _,
    esp_backtrace as _,
};

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

const WIFI_NETWORK: &str = env!("WIFI_NETWORK");
const WIFI_PASSWORD: &str = env!("WIFI_PASSWORD");
const TCP_PORT: u16 = 1234;
const REMOTE_ENDPOINT: (Ipv4Addr, u16) = (Ipv4Addr::new(192, 168, 65, 93), TCP_PORT);
const CLIENT_NAME: &str = "ESP32-C3";

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static, WifiStaDevice>>) {
    runner.run().await
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    log::info!("Start Connection Task");
    println!("Device capabilities: {:?}", controller.capabilities());
    loop {
        match esp_wifi::wifi::wifi_state() {
            WifiState::StaConnected => {
                // wait until we're no longer connected
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_millis(5000)).await
            }
            _ => {}
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = Configuration::Client(ClientConfiguration {
                ssid: WIFI_NETWORK.try_into().unwrap(),
                password: WIFI_PASSWORD.try_into().unwrap(),
                channel: Some(5),
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            log::info!("Starting wifi");
            controller.start_async().await.unwrap();
            log::info!("Wifi started!");
        }
        println!("About to connect...");

        match controller.connect_async().await {
            Ok(_) => log::info!("Wifi connected!"),
            Err(e) => {
                log::warn!("Failed to connect to wifi: {e:?}");
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let mut rng = Rng::new(peripherals.RNG);

    let init = &*mk_static!(
        EspWifiController<'static>,
        init(timg0.timer0, rng.clone(), peripherals.RADIO_CLK).unwrap()
    );

    let wifi = peripherals.WIFI;
    let (wifi_interface, controller) =
        esp_wifi::wifi::new_with_mode(&init, wifi, WifiStaDevice).unwrap();

    let timg1 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timg1.timer0);

    let mut dhcp_config = DhcpConfig::default();
    dhcp_config.hostname = Some(heapless::String::from_str(CLIENT_NAME).unwrap());
    let config = Config::dhcpv4(dhcp_config);

    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    // Init network stack
    let (stack, runner) = embassy_net::new(
        wifi_interface,
        config,
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );

    spawner.must_spawn(connection(controller));
    spawner.must_spawn(net_task(runner));

    log::info!("Waiting for DHCP...");
    while !stack.is_config_up() {
        Timer::after_millis(100).await;
    }
    log::info!("DHCP is Now Up!");

    match stack.config_v4(){
        Some(value) => {
            log::info!("Server Address: {:?}", value.address.address());
            log::info!("Gateway {:?}", value.gateway);
            log::info!("DNS Server {:?}", value.dns_servers);
        },
        None => {
            log::warn!("Unable to Get the Adrress");
        }
    };

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut buf = [0; 1024];

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);     

        match stack.config_v4(){
            Some(value) => {
                log::info!("Server Address: {:?}", value.address.address());
                log::info!("Gateway {:?}", value.gateway);
                log::info!("DNS Server {:?}", value.dns_servers);
            },
            None => {
                log::warn!("Unable to Get the Adrress");
            }
        };

        log::info!("Connecting to the server {:?}...", REMOTE_ENDPOINT);
        match with_timeout(Duration::from_secs(5), socket.connect(REMOTE_ENDPOINT)).await {
            Ok(value) => {
                match value {
                    Ok(()) => {
                        socket.set_timeout(Some(Duration::from_secs(5)));
                    }
                    Err(e) => {
                        log::warn!("Connect Error: {:?}", e);
                        Timer::after(Duration::from_millis(1000)).await;
                        continue;
                    }
                }
            }
            Err(_) => {
                log::warn!("No Connection after 5s");
                continue;
            }
        }

        log::info!("Connected to Endpoint!");

        loop {
            let r = socket.write_all(b"Hello").await;

            if let Err(e) = r {
                println!("write error: {:?}", e);
                break;
            }
            
            let n = match socket.read(&mut buf).await {
                Ok(0) => {
                    log::info!("read EOF");
                    break;
                }
                Ok(n) => n,
                Err(e) => {
                    log::warn!("read error: {:?}", e);
                    break;
                }
            };

            println!("{}", core::str::from_utf8(&buf[..n]).unwrap());
            Timer::after(Duration::from_millis(1000)).await;
        }
    }
}