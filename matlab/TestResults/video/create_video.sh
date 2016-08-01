# changing -r modify the frame rate of the video
ffmpeg -r 17 -i %04d.png  -f mp4 -q:v 0 -vcodec mpeg4 -r 17 out.mp4
