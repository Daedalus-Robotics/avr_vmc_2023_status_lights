from threading import Timer
from typing import Tuple, List

import board
import neopixel_spi
import rclpy
from rclpy.node import Node
from avr_vmc_2023_status_interfaces.srv import SetLight, FlashLight


class StatusLightsNode(Node):

    def __init__(self) -> None:
        super().__init__('status_lights', namespace="status_lights")

        self.declare_parameter('led_count', 8)

        self.led_count = self.get_parameter('led_count').value

        self.spi = board.SPI()
        self.pixels = neopixel_spi.NeoPixel_SPI(self.spi, self.led_count)

        self.flash_timers: List[Timer | None] = [None] * self.led_count

        self.set_srv = self.create_service(
            SetLight,
            'set_color',
            self._set_color_callback
        )
        self.flash_srv = self.create_service(
            FlashLight,
            'flash_color',
            self._flash_color_callback
        )

        self.show_timer = self.create_timer(
            callback=self.pixels.show,
            timer_period_sec=0.1,
        )

    def _set_color_callback(self, request: SetLight, response: SetLight) -> SetLight:
        self.set_color(
            led_num=request.led_num,
            color=(request.color.r, request.color.g, request.color.b)
        )

        return response

    def _flash_color_callback(self, request: FlashLight, response: FlashLight) -> FlashLight:
        self.flash_color(
            led_num=request.led_num,
            color=(request.color.r, request.color.g, request.color.b),
            timeout=request.timeout
        )

        return response

    def set_color(self, led_num: int, color: Tuple[int, int, int]) -> None:
        """
        This method sets the color of the passed number led to the passed color

        :param led_num:
        :param color: Tuple[int, int, int]
        :return:
        """

        if 0 <= led_num < self.led_count:
            if self.flash_timers[led_num] is not None:
                self.flash_timers[led_num].cancel()
                self.flash_timers[led_num] = None
            self.pixels[led_num] = color
            self.pixels.show()

    def flash_color(self, led_num: int, color: Tuple[int, int, int], timeout: float) -> None:
        """
        This method is for automation or any other game strategy related cues. e.

        :param led_num:
        :param color: (r, g, b) color
        :param timeout:
        """

        if 0 <= led_num < self.led_count:
            old_color = self.pixels[led_num]
            self.set_color(led_num, color)
            self.flash_timers[led_num] = Timer(timeout, lambda: self.set_color(led_num, old_color))
            self.flash_timers[led_num].start()


def main() -> None:
    rclpy.init()
    node = StatusLightsNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
