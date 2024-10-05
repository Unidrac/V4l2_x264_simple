#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>              /* low-level I/O */
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

#include <SDL2/SDL.h>
#include <x264.h>
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>

struct buffer {
    void   *start;
    size_t length;
};

int main(int argc, char **argv) {
    int fd;
    const char *dev_name = "/dev/video4";

    // Open the video device
    fd = open(dev_name, O_RDWR | O_NONBLOCK, 0);
    if (fd == -1) {
        perror("Opening video device");
        return 1;
    }

    // Set the video format
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));

    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = 1280;
    fmt.fmt.pix.height      = 720;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV; // YUYV format
    fmt.fmt.pix.field       = V4L2_FIELD_ANY;

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
        perror("Setting Pixel Format");
        return 1;
    }

    // Request buffers
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));

    req.count  = 1; // Request only one buffer
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;


    if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
        perror("Requesting Buffer");
        return 1;
    }

    // Map the buffers
    struct buffer *buffers = calloc(req.count, sizeof(*buffers));

    for (int i = 0; i < req.count; ++i) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));

        buf.type    = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory  = V4L2_MEMORY_MMAP;
        buf.index   = i;

        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) {
            perror("Querying Buffer");
            return 1;
        }

        buffers[i].length = buf.length;
        buffers[i].start  = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);

        if (buffers[i].start == MAP_FAILED) {
            perror("Buffer Map Error");
            return 1;
        }
    }

    // Queue the buffers
    for (int i = 0; i < req.count; ++i) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));

        buf.type    = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory  = V4L2_MEMORY_MMAP;
        buf.index   = i;

        if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
            perror("Queue Buffer");
            return 1;
        }
    }

    // Start streaming
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (ioctl(fd, VIDIOC_STREAMON, &type) == -1) {
        perror("Start Capture");
        return 1;
    }

    // Initialize x264 encoder
    x264_param_t param;
    x264_param_default_preset(&param, "veryfast", "zerolatency");
    // Set rate control parameters
    param.rc.i_rc_method = X264_RC_ABR; // Use Average Bitrate mode
    param.rc.i_bitrate   = 2000;        // Set target bitrate to 2000 kbps

    // Optional: Set VBV parameters to control bitrate peaks
    param.rc.i_vbv_buffer_size = 2000;  // Buffer size in kbps
    param.rc.i_vbv_max_bitrate = 5000;  // Max bitrate in kbps


    param.i_threads       = 1;
    param.i_width         = fmt.fmt.pix.width;
    param.i_height        = fmt.fmt.pix.height;
    param.i_fps_num       = 60;
    param.i_fps_den       = 1;
    param.i_keyint_max    = 30;
    param.b_intra_refresh = 1;
    param.b_annexb        = 1;
    param.i_csp           = X264_CSP_I420;

    x264_param_apply_profile(&param, "baseline");

    x264_t *encoder = x264_encoder_open(&param);

    if (!encoder) {
        fprintf(stderr, "Failed to open encoder\n");
        return 1;
    }

    // Initialize FFmpeg decoder
    // avcodec_register_all();

    AVCodec *dec_codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!dec_codec) {
        fprintf(stderr, "Codec not found\n");
        return 1;
    }

    AVCodecContext *dec_ctx = avcodec_alloc_context3(dec_codec);
    if (!dec_ctx) {
        fprintf(stderr, "Could not allocate video codec context\n");
        return 1;
    }

    if (avcodec_open2(dec_ctx, dec_codec, NULL) < 0) {
        fprintf(stderr, "Could not open codec\n");
        return 1;
    }

    // Initialize SwsContext for color space conversion
    struct SwsContext *sws_ctx = sws_getContext(
        fmt.fmt.pix.width, fmt.fmt.pix.height, AV_PIX_FMT_YUYV422,
        fmt.fmt.pix.width, fmt.fmt.pix.height, AV_PIX_FMT_YUV420P,
        SWS_BILINEAR, NULL, NULL, NULL);

    if (!sws_ctx) {
        fprintf(stderr, "Cannot initialize the conversion context\n");
        return 1;
    }

    // Allocate frames
    AVFrame *frameYUYV = av_frame_alloc();
    AVFrame *frameYUV  = av_frame_alloc();

    frameYUYV->format = AV_PIX_FMT_YUYV422;
    frameYUYV->width  = fmt.fmt.pix.width;
    frameYUYV->height = fmt.fmt.pix.height;

    frameYUV->format = AV_PIX_FMT_YUV420P;
    frameYUV->width  = fmt.fmt.pix.width;
    frameYUV->height = fmt.fmt.pix.height;

    av_image_alloc(frameYUYV->data, frameYUYV->linesize, frameYUYV->width, frameYUYV->height, frameYUYV->format, 32);
    av_image_alloc(frameYUV->data, frameYUV->linesize, frameYUV->width, frameYUV->height, frameYUV->format, 32);

    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "Could not initialize SDL - %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window *window = SDL_CreateWindow("V4L2 Capture",
        SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
        fmt.fmt.pix.width, fmt.fmt.pix.height, 0);

    if (!window) {
        fprintf(stderr, "SDL: could not create window - exiting\n");
        exit(1);
    }

    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    SDL_Texture *texture   = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_IYUV, SDL_TEXTUREACCESS_STREAMING, fmt.fmt.pix.width, fmt.fmt.pix.height);
    SDL_SetHint(SDL_HINT_RENDER_VSYNC, "0");

    // Main loop
// Main loop
uint64_t total_encoded_bytes = 0;
uint64_t last_time = 0;

for (;;) {
    // Dequeue buffer
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));

    buf.type    = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory  = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_DQBUF, &buf) == -1) {
        if (errno == EAGAIN) {
            usleep(10000);
            continue;
        } else {
            perror("Dequeue Buffer");
            return 1;
        }
    }

    // Copy data to frameYUYV
    memcpy(frameYUYV->data[0], buffers[buf.index].start, buf.bytesused);

    // Convert to YUV420P
    sws_scale(sws_ctx, (const uint8_t * const *)frameYUYV->data, frameYUYV->linesize, 0,
              fmt.fmt.pix.height, frameYUV->data, frameYUV->linesize);

    // Prepare x264 picture
    x264_picture_t pic_in, pic_out;
    x264_picture_init(&pic_in);

    pic_in.img.i_csp    = X264_CSP_I420;
    pic_in.img.i_plane  = 3;
    pic_in.img.i_stride[0] = frameYUV->linesize[0];
    pic_in.img.i_stride[1] = frameYUV->linesize[1];
    pic_in.img.i_stride[2] = frameYUV->linesize[2];
    pic_in.img.plane[0] = frameYUV->data[0];
    pic_in.img.plane[1] = frameYUV->data[1];
    pic_in.img.plane[2] = frameYUV->data[2];

    // Encode frame
    x264_nal_t *nals;
    int i_nals;
    int frame_size = x264_encoder_encode(encoder, &nals, &i_nals, &pic_in, &pic_out);

    if (frame_size < 0) {
        fprintf(stderr, "Error encoding frame\n");
        return 1;
    }

    if (frame_size > 0) {
        // Update total encoded bytes
        total_encoded_bytes += frame_size;

        // Get current time in seconds
        uint64_t current_time = SDL_GetTicks64() / 1000; // SDL_GetTicks64() returns milliseconds

        // Initialize last_time if it's the first frame
        if (last_time == 0) {
            last_time = current_time;
        }

        // Calculate bitrate every second
        if (current_time > last_time) {
            uint64_t elapsed_time = current_time - last_time;

            // Calculate bitrate in kilobits per second (kbps)
            double bitrate = (total_encoded_bytes * 8.0) / (elapsed_time * 1000.0);

            printf("Current Bitrate: %.2f kbps\n", bitrate);

            // Reset counters
            total_encoded_bytes = 0;
            last_time = current_time;
        }

        // Decode frame
        AVPacket packet;
        av_init_packet(&packet);

        packet.data = nals[0].p_payload;
        packet.size = frame_size;

        if (avcodec_send_packet(dec_ctx, &packet) < 0) {
            fprintf(stderr, "Error sending packet for decoding\n");
            return 1;
        }

        AVFrame *dec_frame = av_frame_alloc();
        if (avcodec_receive_frame(dec_ctx, dec_frame) == 0) {
            // Update SDL texture
            SDL_UpdateYUVTexture(texture, NULL,
                dec_frame->data[0], dec_frame->linesize[0],
                dec_frame->data[1], dec_frame->linesize[1],
                dec_frame->data[2], dec_frame->linesize[2]);

            SDL_RenderClear(renderer);
            SDL_RenderCopy(renderer, texture, NULL, NULL);
            SDL_RenderPresent(renderer);

            av_frame_unref(dec_frame);
        }

        av_frame_free(&dec_frame);
    }

    // Re-queue the buffer
    if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
        perror("Queue Buffer");
        return 1;
    }

    // Handle SDL events
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) {
            goto cleanup;
        }
    }
}

cleanup:
    // Cleanup
    x264_encoder_close(encoder);
    avcodec_free_context(&dec_ctx);
    sws_freeContext(sws_ctx);

    av_freep(&frameYUYV->data[0]);
    av_freep(&frameYUV->data[0]);
    av_frame_free(&frameYUYV);
    av_frame_free(&frameYUV);

    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    // Stop streaming
    if (ioctl(fd, VIDIOC_STREAMOFF, &type) == -1) {
        perror("Stop Capture");
        return 1;
    }

    // Unmap buffers
    for (int i = 0; i < req.count; ++i) {
        munmap(buffers[i].start, buffers[i].length);
    }

    free(buffers);
    close(fd);

    return 0;
}
