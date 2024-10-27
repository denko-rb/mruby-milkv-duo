module WiringX
  class AnalogInput
    ADC_PATH = "/sys/class/cvi-saradc/cvi-saradc0/device/cv_saradc"

    # Map GPIOS (keys) to their SARADC channels (values).
    VALID_GPIOS = {
      26 => 1,
      27 => 2,
    }

    attr_reader :pin, :driver_path

    def initialize(pin)
      raise "No ADC for pin #{pin}" unless VALID_GPIOS.keys.include?(pin)
      @pin = pin

      load_driver
    end

    def read
      # Make sure to use f.print, not f.puts here. Doesn't work if ending in newline.
      File.open(ADC_PATH, "w") { |f| f.print "#{VALID_GPIOS[pin]}" }
      File.read(ADC_PATH).to_i
    end

    def load_driver
      return if driver_loaded

      puts "SARADC driver not loaded. Searching..."
      find_driver
      raise "No SARADC driver found!" unless driver_path
      puts "Loading SARADC driver: #{driver_path}"

      result = `insmod #{driver_path}`
    end

    def driver_loaded
      result = `lsmod | grep -E "cv180x_saradc|cv181x_saradc"`
      result = false if result.empty?
      result
    end

    def find_driver
      result = `find / -name "cv180x_saradc.ko" 2>/dev/null`
      result = `find / -name "cv181x_saradc.ko" 2>/dev/null` if result.empty?
      result = nil if result.empty?
      @driver_path = result
    end
  end
end
