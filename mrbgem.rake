require_relative "mrblib/version"

MRuby::Gem::Specification.new('mruby-milkv-duo') do |spec|
  spec.license = 'MIT'
  spec.authors = 'vickash'
  spec.version = Duo::VERSION

  # Include files in the right order.
  spec.rbfiles = [
    "#{dir}/mrblib/version.rb",
  ]
  duo_mrblib_files = Dir.glob("#{dir}/mrblib/duo/*")
  spec.rbfiles += duo_mrblib_files
end
