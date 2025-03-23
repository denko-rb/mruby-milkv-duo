module Duo
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
      self.polarity = :normal
      enable
      self.duty = 0
    end

    def polarity=(p=:normal)
      if p == :inversed
        Duo.pwm_set_polarity(pin, 1)
        @polarity = :inversed
      else
        Duo.pwm_set_polarity(pin, 0)
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
      Duo.pwm_set_period(pin, p)
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
      Duo.pwm_set_duty(pin, d_ns)
      @duty = d_ns
    end

    def disable
      Duo.pwm_enable(pin, 0)
      @enabled = false
    end

    def enable
      Duo.pwm_enable(pin, 1)
      @enabled = true
    end
  end
end
