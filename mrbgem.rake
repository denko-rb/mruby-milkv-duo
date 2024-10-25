require_relative "mrblib/version"

MRuby::Gem::Specification.new('mruby-milkv-wiringx') do |spec|
  spec.license = 'MIT'
  spec.authors = 'vickash'
  spec.version = WiringX::VERSION

  # Include files in the right order.
  spec.rbfiles = [
    "#{dir}/mrblib/version.rb",
    "#{dir}/mrblib/wiringx/hardware_pwm.rb",
    "#{dir}/mrblib/wiringx/positional_servo.rb",
    "#{dir}/mrblib/wiringx/infrared.rb",
  ]
end
