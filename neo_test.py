import board
import neopixel

pixels = neopixel.NeoPixel(board.D18, 10)
pixels.fill((0, 255, 0))
pixels[0] = (255,0,0)
pixels.show()

print("Done")