#pragma once

void bitbangSetup(uint8_t port, uint8_t pin);
void bitbangWs2812(uint8_t ledCount, __xdata uint8_t * ledData);
void bitbangSk6812_2020(uint8_t ledCount, __xdata uint8_t * ledData);
