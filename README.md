Camera -> V4l2 -> H.264 Encode -> H.264 Decode -> SDL Window

compile command:

gcc -o test camera_sdl2.c -lx264 -lavcodec -lavutil -lswscale -lSDL2

usage:

./test

generate by ChatGPT o1-preview
