module WiringX
  class HardwarePWM
    NS_PER_S  = 10**9
    NS_PER_US = 10**3

    attr_reader :pin, :period, :duty, :polarity, :enabled

    def initialize(pin, frequency: nil, period: nil)
      @pin = pin

      # Accept either frequency (in Hz) or period in nanoseconds.
      if (frequency && period) || (!frequency && !period)
        raise "either period: or frequency: is required, but not both"
      end

      period ? self.period = period : self.frequency = frequency

      # Default to with 0 duty cycle and normal polarity.
      self.duty = 0
      self.polarity = :normal

      enable
    end

    def polarity=(p=:normal)
      if p == :inversed
        pwm_set_polarity(pin, 1)
        @polarity = :inversed
      else
        pwm_set_polarity(pin, 0)
        @polarity = :normal
      end
    end

    def frequency=(freq)
      self.period = (NS_PER_S / freq.to_f).round
      @frequency = freq
    end

    def frequency
      # If not set explicitly, calculate from period, rounded to nearest Hz.
      @frequency ||= (NS_PER_S / period.to_f).round
    end

    def period=(p)
      pwm_set_duty(pin, 0) unless duty == 0
      pwm_set_period(pin, p)
      @frequency = nil
      @period = p
    end

    def duty_percent
      return 0.0 if (!duty || !period) || (duty == 0)
      (duty / period.to_f) * 100.0
    end

    def duty_percent=(d)
      raise "duty_cycle: #{d} % cannot be more than 100%" if d > 100
      d_ns = ((d / 100.0) * @period.to_i).round
      self.duty = d_ns
    end

    def duty_us=(d_us)
      d_ns = (d_us * NS_PER_US).round
      self.duty = d_ns
    end

    def duty=(d_ns)
      raise "duty cycle: #{d_ns} ns cannot be longer than period: #{period} ns" if d_ns > period
      pwm_set_duty(pin, d_ns)
      @duty = d_ns
    end

    def disable
      pwm_enable(pin, 0)
      @enabled = false
    end

    def enable
      pwm_enable(pin, 1)
      @enabled = true
    end
  end
end
