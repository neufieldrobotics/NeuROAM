#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<EOF
Usage: $0 [--verify] [--mirror] <SRC_DIR> <TGT_ROOT>

Copy entire directory tree to TGT_ROOT/<basename(SRC_DIR)> with speed & safety.

Examples:
  $0 /data/rosbags /media/neuroam/ext_ssd
  # -> /media/neuroam/ext_ssd/rosbags

Options:
  --mirror   Make destination subdir an exact mirror of source (deletes extras).
  --verify   After copy, verify (size+mtime fast pass) and sha256 (slow).
EOF
}

DO_VERIFY=0
DO_MIRROR=0
ARGS=()
while (( "$#" )); do
  case "${1:-}" in
    -h|--help) usage; exit 0 ;;
    --mirror) DO_MIRROR=1; shift ;;
    --verify) DO_VERIFY=1; shift ;;
    --) shift; break ;;
    -* ) echo "Unknown option: $1" >&2; usage; exit 1 ;;
    *  ) ARGS+=("$1"); shift ;;
  esac
done

if [ "${#ARGS[@]}" -ne 2 ]; then
  usage; exit 1
fi

SRC="${ARGS[0]%/}"
TGT_ROOT="${ARGS[1]%/}"

[ -d "$SRC" ] || { echo "Source not found or not a directory: $SRC" >&2; exit 1; }
mkdir -p "$TGT_ROOT"

BASENAME="$(basename "$SRC")"
DST="$TGT_ROOT/$BASENAME"
mkdir -p "$DST"

echo "Source:           $SRC"
echo "Target root:      $TGT_ROOT"
echo "Destination dir:  $DST"

# rsync tuned for fast local disk→disk copies:
RSYNC_FLAGS=(-aW --inplace --preallocate --no-compress --human-readable --info=progress2)
# Note: trailing slashes matter: SRC/ -> copies contents into DST/
# (We use that so the resulting path is exactly .../BASENAME/<contents>)

if [ "$DO_MIRROR" -eq 1 ]; then
  RSYNC_FLAGS+=(--delete --delete-after --delete-excluded)
  echo "Mirror mode: on (destination extras will be deleted)"
fi

start_ts=$(date +%s)
rsync "${RSYNC_FLAGS[@]}" "$SRC"/ "$DST"/
sync
end_ts=$(date +%s)

# Throughput report
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
  tmp_src=$(mktemp); tmp_dst=$(mktemp)
  (cd "$SRC" && find . -type f -print0 | xargs -0 stat -c '%n|%s|%Y' | sort -u) >"$tmp_src"
  (cd "$DST" && find . -type f -print0 | xargs -0 stat -c '%n|%s|%Y' | sort -u) >"$tmp_dst"
  if ! diff -u "$tmp_src" "$tmp_dst" >/dev/null; then
    echo "Fast verify mismatch (size/mtime differs for some files)"; rm -f "$tmp_src" "$tmp_dst"; exit 2
  fi
  rm -f "$tmp_src" "$tmp_dst"
  echo "Fast verify OK."

  echo "==> sha256 verification (slow)..."
  (cd "$SRC" && find . -type f -print0 | sort -z | xargs -0 -I{} sha256sum "{}" | sort) > /tmp/src.sha
  (cd "$DST" && find . -type f -print0 | sort -z | xargs -0 -I{} sha256sum "{}" | sort) > /tmp/dst.sha
  if ! diff -u /tmp/src.sha /tmp/dst.sha >/dev/null; then
    echo "SHA256 mismatch!"; rm -f /tmp/src.sha /tmp/dst.sha; exit 3
  fi
  rm -f /tmp/src.sha /tmp/dst.sha
  echo "sha256 verification OK."
fi

echo "All done."
