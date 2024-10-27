module Duo
  class Infrared < HardwarePWM
    FREQUENCY = 38_000
    DUTY = 33.333

    def initialize(*args, **kwargs)
      new_kwargs = {frequency: FREQUENCY}.merge(kwargs)
      super(*args, **new_kwargs)
      disable
    end

    def transmit(pulses, duty: DUTY)
      self.duty_percent = duty
      Duo.tx_wave_ook(pin, @duty, pulses)
    end
  end
end
