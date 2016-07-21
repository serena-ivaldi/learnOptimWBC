# changing -r modify the frame rate of the video
ffmpeg -r 20 -i %04d.png  -f mp4 -q:v 0 -vcodec mpeg4 -r 20 out.mp4
