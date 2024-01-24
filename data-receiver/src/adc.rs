use embassy_stm32::{
    adc::{AdcPin, Instance, Resolution, SampleTime, ADC_POWERUP_TIME_US},
    into_ref,
    time::Hertz,
    Peripheral, PeripheralRef,
};

/// Analog to Digital driver.
pub struct Adc<'d, T: Instance> {
    #[allow(unused)]
    adc: PeripheralRef<'d, T>,
    sample_time: SampleTime,
}

enum Prescaler {
    Div2,
    Div4,
    Div6,
    Div8,
}

impl Prescaler {
    fn from_pclk2(freq: Hertz) -> Self {
        // Datasheet for both F4 and F7 specifies min frequency 0.6 MHz, typ freq. 30 MHz and max 36 MHz.
        const MAX_FREQUENCY: Hertz = Hertz(36_000_000);
        let raw_div = freq.0 / MAX_FREQUENCY.0;
        match raw_div {
            0..=1 => Self::Div2,
            2..=3 => Self::Div4,
            4..=5 => Self::Div6,
            6..=7 => Self::Div8,
            _ => panic!(
                "Selected PCLK2 frequency is too high for ADC with largest possible prescaler."
            ),
        }
    }

    fn adcpre(&self) -> embassy_stm32::pac::adccommon::vals::Adcpre {
        match self {
            Prescaler::Div2 => embassy_stm32::pac::adccommon::vals::Adcpre::DIV2,
            Prescaler::Div4 => embassy_stm32::pac::adccommon::vals::Adcpre::DIV4,
            Prescaler::Div6 => embassy_stm32::pac::adccommon::vals::Adcpre::DIV6,
            Prescaler::Div8 => embassy_stm32::pac::adccommon::vals::Adcpre::DIV8,
        }
    }
}

impl<'d, T> Adc<'d, T>
where
    T: Instance,
{
    pub fn new(
        adc: impl Peripheral<P = T> + 'd,
        delay: &mut impl embedded_hal::delay::DelayNs,
    ) -> Self {
        into_ref!(adc);
        T::enable_and_reset();

        let presc = Prescaler::from_pclk2(T::frequency());
        T::common_regs()
            .ccr()
            .modify(|w| w.set_adcpre(presc.adcpre()));
        T::regs().cr2().modify(|reg| {
            reg.set_adon(true);
        });

        delay.delay_us(ADC_POWERUP_TIME_US);

        Self {
            adc,
            sample_time: Default::default(),
        }
    }

    pub fn set_sample_time(&mut self, sample_time: SampleTime) {
        self.sample_time = sample_time;
    }

    pub fn set_resolution(&mut self, resolution: Resolution) {
        T::regs().cr1().modify(|reg| reg.set_res(resolution.into()));
    }

    pub fn set_running(&mut self, pin: &mut impl AdcPin<T>) {
        pin.set_as_analog();

        // Configure ADC
        let channel = pin.channel();

        // Select channel
        T::regs().sqr3().write(|reg| reg.set_sq(0, channel));

        // Configure channel
        Self::set_channel_sample_time(channel, self.sample_time);

        // clear end of conversion flag
        T::regs().sr().modify(|reg| {
            reg.set_eoc(false);
        });

        // Start conversion
        T::regs().cr2().modify(|reg| {
            reg.set_swstart(true);
            reg.set_dma(false);
            reg.set_eocs(embassy_stm32::pac::adc::vals::Eocs::EACHSEQUENCE);
        });

        while T::regs().sr().read().strt() == false {
            // spin //wait for actual start
        }
    }

    pub fn read(&mut self) -> u16 {
        T::regs().dr().read().0 as u16
    }

    fn set_channel_sample_time(ch: u8, sample_time: SampleTime) {
        let sample_time = sample_time.into();
        if ch <= 9 {
            T::regs()
                .smpr2()
                .modify(|reg| reg.set_smp(ch as _, sample_time));
        } else {
            T::regs()
                .smpr1()
                .modify(|reg| reg.set_smp((ch - 10) as _, sample_time));
        }
    }
}

impl<'d, T: Instance> Drop for Adc<'d, T> {
    fn drop(&mut self) {
        T::regs().cr2().modify(|reg| {
            reg.set_adon(false);
        });

        T::disable();
    }
}
