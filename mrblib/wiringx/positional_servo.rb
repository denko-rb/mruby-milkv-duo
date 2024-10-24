module WiringX
  class PositionalServo
    FREQUENCY = 50
    NANOSECONDS = 1_000_000_000

    attr_reader :angle, :pin

    def initialize(pin, min_us, max_us, min_angle, max_angle)
      @pin = pin
      WiringX.pwm_set_period(pin, NANOSECONDS / FREQUENCY)
      WiringX.pwm_set_duty(pin, 0)
      WiringX.pwm_set_polarity(pin, 0)
      WiringX.pwm_enable(pin, 1)

      raise "min_us: #{min_us} cannot be lower than max_us: #{max_us}" if max_us < min_us
      @min_us = min_us
      @max_us = max_us
      @us_range = @max_us - @min_us

      @min_angle = min_angle
      @max_angle = max_angle
    end

    def angle=(a)
      ratio = (a - @min_angle).to_f / (@max_angle - @min_angle)
      raise "angle: #{a} outside servo range" if (ratio < 0) || (ratio > 1)

      d_ns = ((@us_range * ratio) + @min_us) * 1000
      WiringX.pwm_set_duty(pin, d_ns)
      @angle = a
    end
  end
end
