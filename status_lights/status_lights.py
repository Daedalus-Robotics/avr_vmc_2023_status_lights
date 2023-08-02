import rclpy
import rclpy.node

import board
import neopixel_spi

class StatusLightsNode(Node):

    def __init__(self) -> None:
        super().__init__('status_lights')

        self.declare_parameter('led_count', 50)

        led_count = self.get_parameter('my_parameter').value()

        # TODO: Initialize Neopixel stuff here
        self.spi = board.SPI()
        self.pixels = neopixel_spi.NeoPixel_SPI(self.spi, led_count)


def main() -> None:
    rclpy.init()
    node = StatusLightsNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
