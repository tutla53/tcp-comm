/*  Servo Task 
    - Controlling Servo MG996R by using the PIO
    - Commanded Angle will be sent from the TCP at the Main Task
*/

use {
    core::time::Duration,
    rp2040_servo_pio::ServoPioBuilder,
    crate::resources::gpio_list::{
        Irqs,
        ServoPioResources,
    },
    embassy_rp::{
        pio::Pio,
        pio_programs::{
            pwm::{PioPwm, PioPwmProgram},
        },
    },
    embassy_time::Timer,
    embassy_sync::{
        signal::Signal,
        blocking_mutex::raw::CriticalSectionRawMutex,
    },
    {defmt_rtt as _, panic_probe as _},
};

const REFRESH_INTERVAL: u64 = 20000;
static DRIVE_CONTROL: Signal<CriticalSectionRawMutex, Command> = Signal::new();

pub enum Command {
    Left(i16),
    Right(i16),
    Up(i16),
    Down(i16)
}

pub fn send_command(command: Command) {
    DRIVE_CONTROL.signal(command);
}

async fn wait_command() -> Command {
    DRIVE_CONTROL.wait().await
}

#[embassy_executor::task]
pub async fn servo_pio(r: ServoPioResources) {
    let Pio { mut common, sm0, sm1, .. } = Pio::new(r.SERVO_PIO_CH, Irqs);
    let prg = PioPwmProgram::new(&mut common);

    let body_pwm_pio = PioPwm::new(&mut common, sm0, r.SERVO_BODY_PIN, &prg);
    let head_pwm_pio = PioPwm::new(&mut common, sm1, r.SERVO_HEAD_PIN, &prg);

    let mut body_servo = ServoPioBuilder::new(body_pwm_pio)
        .set_period(Duration::from_micros(REFRESH_INTERVAL))
        .set_max_degree_rotation(180)
        .set_min_pulse_width(Duration::from_micros(1000))
        .set_max_pulse_width(Duration::from_micros(2000))
        .build();

    let mut head_servo = ServoPioBuilder::new(head_pwm_pio)
        .set_period(Duration::from_micros(REFRESH_INTERVAL))
        .set_max_degree_rotation(180)
        .set_min_pulse_width(Duration::from_micros(1000))
        .set_max_pulse_width(Duration::from_micros(2000))
        .build();

    body_servo.start();
    head_servo.start();
    Timer::after_secs(1).await;

    body_servo.rotate(90);
    head_servo.rotate(90);

    let mut head_degree: i16 = 0;
    let mut body_degree: i16 = 0;

    loop {
        let command = wait_command().await;
        
        match command {
            Command::Up(inc) => {
                head_degree = head_degree + inc;
                log::info!("Up {}", head_degree);
            },
            Command::Down(inc) => {
                head_degree = head_degree - inc;
                log::info!("Down {}", head_degree);
            },
            Command::Left(inc) => {
                body_degree = body_degree + inc;
                log::info!("Left {}", body_degree);
            },
            Command::Right(inc) => {
                body_degree = body_degree - inc;
                log::info!("Right {}", body_degree);
            }
        }
        
        if head_degree<0 {head_degree = 0;}
        else if head_degree>180{head_degree = 180;}
        
        if body_degree<0 {body_degree = 0;}
        else if body_degree>180{body_degree = 180;}

        body_servo.rotate(body_degree as u64);
        head_servo.rotate(head_degree as u64);
    }
}