import rclpy
from rclpy.node import Node

import board
import neopixel_spi
from threading import Timer
from typing import Tuple

class StatusLightsNode(Node):

    def __init__(self) -> None:
        super().__init__('status_lights')

        self.declare_parameter('led_count', 8)

        led_count = self.get_parameter('led_count').value

        self.spi = board.SPI()
        self.pixels = neopixel_spi.NeoPixel_SPI(self.spi, led_count)

        self.show_timer = self.create_timer(
            callback = self.pixels.show,
            timer_period_sec = 0.1,
        )

    def set_color(self, led_num: int, color: Tuple[int, int, int]) -> None:
        """
        This method sets the color of the passed number led to the passed color

        :param led_num:
        :param color: Tuple[int, int, int]
        :return:
        """

        self.pixels[led_num] = color

    def flash_color(self, led_num: int, color: Tuple[int, int, int], timeout: float) -> None:
        """
        This method is for automation or any other game strategy related cues. e.

        :param led_num:
        :param color: (r, g, b) color
        :param timeout:
        """
        
        old_color = self.pixels[led_num]
        self.set_color(led_num, color)
        self.pixels.show()
        Timer(timeout, lambda: self.set_color(led_num, old_color)).start()


def main() -> None:
    rclpy.init()
    node = StatusLightsNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
