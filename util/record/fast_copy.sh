#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<EOF
Usage: $0 [--verify] [--mirror] <SRC_DIR> <DST_DIR>

Fast local copy of an entire directory tree (all files, including .mcap).
Designed for SSD -> SSD via USB3/4 on Linux.

Options:
  --verify   After copy, verify by size+mtime (fast) then sha256 (slow).
  --mirror   Make DST a mirror of SRC (delete files at DST that aren't in SRC).
             (Default is additive copy: never deletes at DST.)

Examples:
  $0 /data/rosbags /mnt/ext_ssd/rosbags
  $0 --mirror --verify /data/rosbags /mnt/ext_ssd/rosbags
EOF
}

DO_VERIFY=0
DO_MIRROR=0
ARGS=()
while (( "$#" )); do
  case "${1:-}" in
    -h|--help) usage; exit 0 ;;
    --verify) DO_VERIFY=1; shift ;;
    --mirror) DO_MIRROR=1; shift ;;
    --) shift; break ;;
    -* ) echo "Unknown option: $1" >&2; usage; exit 1 ;;
    *  ) ARGS+=("$1"); shift ;;
  esac
done

if [ "${#ARGS[@]}" -ne 2 ]; then
  usage; exit 1
fi

SRC="${ARGS[0]%/}"
DST="${ARGS[1]%/}"
[ -d "$SRC" ] || { echo "Source not found: $SRC" >&2; exit 1; }
mkdir -p "$DST"

echo "Source:      $SRC"
echo "Destination: $DST"

# rsync tuned for local disk→disk:
# -aW              archive + whole-file (no delta) — faster for big files
# --inplace        write directly to dest (avoid temp rename)
# --preallocate    reduce fragmentation on ext4/xfs
# --no-compress    avoid CPU for local copy
# --human-readable nicer stats
# --info=progress2 overall progress
# --delete (optional) if mirroring is requested
RSYNC_FLAGS=(-aW --inplace --preallocate --no-compress --human-readable --info=progress2)
if [ "$DO_MIRROR" -eq 1 ]; then
  RSYNC_FLAGS+=(--delete --delete-after --delete-excluded)
fi

start_ts=$(date +%s)
rsync "${RSYNC_FLAGS[@]}" "$SRC"/ "$DST"/
sync
end_ts=$(date +%s)

# Rough throughput report
total_bytes=$(du -sb "$SRC" | awk '{print $1}')
dur=$(( end_ts - start_ts ))
if (( dur > 0 )); then
  mbps=$(awk -v b="$total_bytes" -v t="$dur" 'BEGIN{print (b/1e6)/t}')
  echo "==> Done in ${dur}s (~"$(printf '%.2f' "$mbps")" MB/s)."
else
  echo "==> Done."
fi

if [ "$DO_VERIFY" -eq 1 ]; then
  echo "==> Verifying size+mtime (fast pass)..."
  # Compare file lists with size+mtime (ignores metadata like ownership)
  tmp_src=$(mktemp); tmp_dst=$(mktemp)
  # Use find to produce stable, relative lists
  (cd "$SRC" && find . -type f -print0 \
    | xargs -0 stat -c '%n|%s|%Y' | sort
