# Video recording

`tellopy/examples/keyboard_and_video.py` shows the live video feed in a window with a telemetry HUD overlaid on top, and can save it to a file.
Press Enter to save a photo, R to start/stop recording a video.
Both are saved to `~/Pictures/` (`%USERPROFILE%\Pictures` on Windows).

## How it works

The drone sends raw H.264 video over UDP.
TelloPy publishes each received chunk as `EVENT_VIDEO_FRAME` without decoding or re-encoding it.
`keyboard_and_video.py` pipes these raw bytes straight into `mencoder`, which writes them into an `.mp4` file as-is (`-ovc copy`, no re-encoding).
This is separate from how the video is decoded for on-screen display (`av` + `opencv-python`, used by this and the other video examples) -- display and recording each read the same packets independently.

## Why mencoder

Piping the raw bytes into an external encoder is simpler than encoding a file in Python, and avoids a re-encoding pass.
`mencoder` is part of the MPlayer project; nothing else in TelloPy needs it (video display does not).

## Installing mplayer (for mencoder)

Only needed if you want to use the recording feature.

### Windows
Download Mplayer for Windows from [SourceForge](https://sourceforge.net/projects/mplayerwin/files/MPlayer-MEncoder/r38151/mplayer-svn-38151-x86_64.7z/download).
Unzip the file via [7zip](https://www.7-zip.org/), and add the extracted folder to your PATH (see [here for env help](https://www.architectryan.com/2018/03/17/add-to-the-path-on-windows-10/)).

### Linux
`$ sudo apt install mplayer mplayer-gui`

### Mac
`$ brew install mplayer`
