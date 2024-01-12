ffmpeg -i ../../../../testdrive/Oz/idle-beep.mp3 -ar 48000 -af 'pan=mono|c0=FL' -f s16be -acodec pcm_s16be - | uqoa_conv > idle-beep.bin
ffmpeg -i ../../../../testdrive/Oz/track-beep.wav -ar 48000 -af 'pan=mono|c0=FL' -f s16be -acodec pcm_s16be - | uqoa_conv > track-beep.bin
ffmpeg -i ../../../../testdrive/Oz/idle-bug.mp3 -ar 48000 -af 'pan=mono|c0=FL' -f s16be -acodec pcm_s16be - | uqoa_conv > idle-bug.bin
ffmpeg -i ../../../../testdrive/Oz/track-bug.wav -ar 48000 -af 'pan=mono|c0=FL' -f s16be -acodec pcm_s16be - | uqoa_conv > track-bug.bin

gcc gen.c -O2 -o gen
./gen idle-beep.bin 1.gdbinit
./gen idle-bug.bin 2.gdbinit

(cd ../..; sh debug/run.sh)
# >> source debug/flash_data/1.gdbinit
# >> source debug/flash_data/2.gdbinit
rm gen 1.gdbinit 2.gdbinit
