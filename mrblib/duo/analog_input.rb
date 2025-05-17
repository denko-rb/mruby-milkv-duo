module Duo
  ADC_PATH = "/sys/class/cvi-saradc/cvi-saradc0/device/cv_saradc"

  def saradc_initialize
    return true if self.saradc_driver_loaded

    puts "SARADC driver not loaded. Searching..."
    driver_path = self.find_saradc_driver
    raise "No SARADC driver found!" unless driver_path

    puts "Loading SARADC driver: #{driver_path}"
    result = `insmod #{driver_path}`

    return true
  end

  def saradc_driver_loaded
    result = `lsmod | grep -E "cv180x_saradc|cv181x_saradc"`
    result = false if result.empty?
    result
  end

  def find_saradc_driver
    result = `find / -name "cv180x_saradc.ko" 2>/dev/null`
    result = `find / -name "cv181x_saradc.ko" 2>/dev/null` if result.empty?
    result = nil if result.empty?
    result
  end

  module_function :saradc_initialize, :saradc_driver_loaded, :find_saradc_driver
end
