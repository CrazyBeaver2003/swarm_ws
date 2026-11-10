#!/usr/bin/env bash
# Explicit local desktop recording; never starts automatically with launch.
set -euo pipefail
if [[ $# -ne 1 ]]; then
  echo 'Usage: record_video.sh NEW_OUTPUT.mp4' >&2
  exit 2
fi
if [[ "${XDG_SESSION_TYPE:-}" == 'wayland' ]]; then
  echo 'Wayland: record the Gazebo/browser window with OBS or the desktop screen recorder.' >&2
  exit 2
fi
if [[ -z "${DISPLAY:-}" ]]; then
  echo 'DISPLAY is empty. Run from the graphical X11 session.' >&2
  exit 2
fi
if ! command -v ffmpeg >/dev/null 2>&1; then
  echo 'ffmpeg is not installed. Install it separately or use OBS/the desktop recorder.' >&2
  exit 2
fi
if [[ -e "$1" ]]; then
  echo 'Output already exists; refusing to overwrite it.' >&2
  exit 2
fi
video_size="${SWARM_VIDEO_SIZE:-}"
if [[ -z "$video_size" ]]; then
  if ! command -v xdpyinfo >/dev/null 2>&1; then
    echo 'Set SWARM_VIDEO_SIZE, for example 1920x1080, or install xdpyinfo.' >&2
    exit 2
  fi
  video_size=$(xdpyinfo -display "$DISPLAY" | awk '/dimensions:/{print $2; exit}')
fi
if [[ ! "$video_size" =~ ^[0-9]+x[0-9]+$ ]]; then
  echo 'Invalid SWARM_VIDEO_SIZE; expected WIDTHxHEIGHT.' >&2
  exit 2
fi
output_dir=$(dirname -- "$1")
mkdir -p -- "$output_dir"
output="$(cd -- "$output_dir" && pwd)/$(basename -- "$1")"
echo "Recording DISPLAY=$DISPLAY at $video_size to $output. Press Ctrl+C to finish."
exec ffmpeg -hide_banner -n -f x11grab -framerate 20 -video_size "$video_size" \
  -i "$DISPLAY" -vf 'pad=ceil(iw/2)*2:ceil(ih/2)*2' \
  -c:v libx264 -preset veryfast -crf 22 -pix_fmt yuv420p "$output"
