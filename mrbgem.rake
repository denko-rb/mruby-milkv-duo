require_relative "mrblib/version"

MRuby::Gem::Specification.new('mruby-milkv-wiringx') do |spec|
  spec.license = 'MIT'
  spec.authors = 'vickash'
  spec.version = WiringX::VERSION

  # Include files in the right order.
  spec.rbfiles = [
    "#{dir}/mrblib/version.rb",
  ]
  wiringx_mrblib_files = Dir.glob("#{dir}/mrblib/wiringx/*")
  spec.rbfiles += wiringx_mrblib_files
end
