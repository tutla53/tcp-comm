//  Resource Allocation Module

use {
    assign_resources::assign_resources,
    embassy_rp::{
        bind_interrupts,
        peripherals,
        pio::InterruptHandler as PioInterruptHandler,
        usb::InterruptHandler as UsbInterruptHandler,
        i2c::InterruptHandler as I2cInterruptHandler,
    },
};

assign_resources! {
    network_resources: NetworkResources {
        CYW43_PWR_PIN: PIN_23,
        CYW43_CS_PIN: PIN_25,
        CYW43_PIO_CH: PIO0,
        CYW43_SPI_DIO: PIN_24,
        CYW43_SPI_CLK: PIN_29,
        CYW43_DMA_CH: DMA_CH0,
    },

    servo_pio_resources: ServoPioResources {
        SERVO_PIO_CH: PIO1,
        SERVO_BODY_PIN: PIN_10,
        SERVO_HEAD_PIN: PIN_12,
    },

    display_resources: DisplayResources {
        I2C_CH: I2C0,
        SCL_PIN: PIN_5,
        SDA_PIN: PIN_4,
    },
}

bind_interrupts!(pub struct Irqs {
    I2C0_IRQ => I2cInterruptHandler<peripherals::I2C0>;
    PIO0_IRQ_0 => PioInterruptHandler<peripherals::PIO0>;
    PIO1_IRQ_0 => PioInterruptHandler<peripherals::PIO1>;
    USBCTRL_IRQ => UsbInterruptHandler<peripherals::USB>;
});

