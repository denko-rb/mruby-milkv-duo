default = (0..3).to_a + (14..22).to_a + (25..27).to_a
default.each do |num|
  `duo-pinmux -w GP#{num}/GP#{num}`
end

# I2C1
`duo-pinmux -w GP4/IIC1_SCL`
`duo-pinmux -w GP5/IIC1_SDA`

# SPI2
`duo-pinmux -w GP6/SPI2_SCK`
`duo-pinmux -w GP7/SPI2_SDO`
`duo-pinmux -w GP8/SPI2_SDI`
`duo-pinmux -w GP9/SPI2_CS_X`

# 2x PWM
`duo-pinmux -w GP10/PWM_10`
`duo-pinmux -w GP11/PWM_11`

# UART0
`duo-pinmux -w GP12/UART0_TX`
`duo-pinmux -w GP13/UART0_RX`
