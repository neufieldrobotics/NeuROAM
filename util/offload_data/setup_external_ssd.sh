#!/usr/bin/env bash
# ============================================================
# External SSD setup for Jetson (ext4, fast + removable-safe)
# - Interactively selects the device
# - (Optionally) repartitions & formats as ext4 (fast settings)
# - Creates a removable-safe /etc/fstab entry
# - Mounts under /media/<user>/<LABEL> for file-manager visibility
# - Applies performance-oriented mount options suitable for USB BOT
# ============================================================

# --- Safety flags ---
# -e : exit on any command error
# -u : error on undefined variables
# -o pipefail : fail a pipeline if any command fails
set -euo pipefail

# ============================================================
# Step 0: Pre-flight checks (commands, privilege, user context)
# ============================================================

# Helper: ensure a required command exists
require_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "Missing required command: $1. Please install it and rerun." >&2
    exit 1
  }
}

# Required tools:
# - lsblk/blkid: list & query block devices
# - parted: partition disks
# - mkfs.ext4: create ext4 filesystem
# - sed/tee: edit fstab safely
# - findmnt: query current mounts
# - mountpoint: test if a path is a mountpoint
for c in lsblk blkid parted mkfs.ext4 sed tee findmnt mountpoint; do require_cmd "$c"; done

# Must run as root (sudo) because we will partition, format, and edit /etc/fstab
if [[ $EUID -ne 0 ]]; then
  echo "Please run with sudo (root). Example: sudo bash $0"
  exit 1
fi

# Determine the *invoking* user so we can set ownership of the mountpoint
# (SUDO_USER is the user who ran sudo; fallback to $USER if absent)
INV_USER="${SUDO_USER:-$USER}"
INV_UID="$(id -u "$INV_USER")"
INV_GID="$(id -g "$INV_USER")"

echo "=== External SSD Setup (ext4, fast + removable-safe) ==="
echo

# ============================================================
# Step 1: Show devices & ask the user to choose the SSD
# ============================================================

echo "Connected block devices:"
# TRAN shows transport (usb, nvme, sata, etc.); HOTPLUG/RM help spot removable/USB devices
lsblk -o NAME,MODEL,SIZE,TRAN,SUBSYSTEMS,HOTPLUG,RM,MOUNTPOINT
echo

read -rp "Enter the DEVICE (e.g., /dev/sda) for the external SSD: " DEV
[[ -b "$DEV" ]] || { echo "Device $DEV not found."; exit 1; }

# Safety: prevent selecting the system root device
SYS_ROOT_DEV="$(findmnt -no SOURCE / || true)"
if [[ "$SYS_ROOT_DEV" == "$DEV"* ]]; then
  echo "ERROR: You selected the system/root device ($DEV). Aborting for safety."
  exit 1
fi

# Informational: removable flag (0=no, 1=yes)
BASENAME="$(basename "$DEV")"
[[ -f "/sys/block/$BASENAME/removable" ]] && \
  echo "Device removable flag: $(</sys/block/$BASENAME/removable) (0=no, 1=yes)."
echo

# Show the current partition table so the user knows what's on the device now
echo "Current partition table for $DEV:"
parted -s "$DEV" print || true
echo

# ============================================================
# Step 2: Decide whether to (re)partition & format the SSD
# ============================================================

read -rp "Do you want to (re)partition and format $DEV as a single ext4 partition? (yes/NO): " DO_FORMAT
DO_FORMAT="${DO_FORMAT,,}"  # normalize to lowercase

# Let the user choose a label that will become the folder name under /media/<user>/
LABEL_DEFAULT="extssd"
read -rp "Enter a filesystem label [default: ${LABEL_DEFAULT}]: " FS_LABEL
FS_LABEL="${FS_LABEL:-$LABEL_DEFAULT}"

if [[ "$DO_FORMAT" == "yes" || "$DO_FORMAT" == "y" ]]; then
  # ------------------------------------------------------------
  # Step 2a: Reformat path (DESTRUCTIVE!)
  # ------------------------------------------------------------
  echo
  echo "!!! WARNING: This will ERASE ALL DATA on $DEV !!!"
  read -rp "Type 'ERASE' to confirm: " CONFIRM
  [[ "$CONFIRM" == "ERASE" ]] || { echo "Confirmation not received. Aborting."; exit 1; }

  echo "Creating GPT and a single primary partition spanning the whole device..."
  parted -s "$DEV" mklabel gpt
  parted -s "$DEV" mkpart primary ext4 0% 100%

  # Ask the kernel to re-read the partition table, then wait for /dev/sdX1 to appear
  partprobe "$DEV" || true
  sleep 2

  PART="${DEV}1"
  [[ -b "$PART" ]] || { echo "Partition $PART not found after creating it."; exit 1; }

  # Format with ext4; disable lazy initialization to avoid first-use slowdowns
  # -E lazy_itable_init=0,lazy_journal_init=0 : initialize metadata eagerly
  # -L <label> : set the filesystem label
  echo "Formatting $PART as ext4 with eager metadata init for better initial write performance..."
  mkfs.ext4 -F -E lazy_itable_init=0,lazy_journal_init=0 -L "$FS_LABEL" "$PART"
else
  # ------------------------------------------------------------
  # Step 2b: Keep existing partitions/filesystem
  # ------------------------------------------------------------
  echo "Skipping format. Will use existing partition & filesystem."
  # Find partitions on the device; prompt if there are multiple
  PARTS=( $(lsblk -ln -o NAME "$DEV" | tail -n +2) )
  if [[ ${#PARTS[@]} -eq 0 ]]; then
    echo "No partitions found on $DEV. You must create one, or rerun and choose to format."
    exit 1
  fi
  if [[ ${#PARTS[@]} -gt 1 ]]; then
    echo "Multiple partitions detected on $DEV:"
    lsblk -ln -o NAME,SIZE,TYPE,MOUNTPOINT "$DEV"
    read -rp "Enter the PARTITION to mount (e.g., ${DEV}1): " PART
  else
    PART="/dev/${PARTS[0]}"
    echo "Using detected partition: $PART"
  fi

  # If there is already a filesystem label, use that (so the mount path matches the current label)
  CUR_LABEL="$(blkid -s LABEL -o value "$PART" || true)"
  FS_LABEL="${CUR_LABEL:-$FS_LABEL}"
fi

# ============================================================
# Step 3: Collect identifiers & choose mount location
# ============================================================

# Get a stable identifier for fstab (UUID is best)
UUID="$(blkid -s UUID -o value "$PART" || true)"
[[ -n "${UUID:-}" ]] || { echo "Could not obtain UUID for $PART. Aborting."; exit 1; }

# Real label (from the FS) if present; otherwise use the chosen label
REAL_LABEL="$(blkid -s LABEL -o value "$PART" || true)"
REAL_LABEL="${REAL_LABEL:-$FS_LABEL}"

echo
echo "Filesystem label: $REAL_LABEL"
echo "UUID:            $UUID"
echo

# Mount under /media/<invoking-user>/<label> so it shows up in the GUI file manager
MOUNT_BASE="/media/$INV_USER"
MOUNT_POINT="$MOUNT_BASE/$REAL_LABEL"

# Ensure base dir exists and is owned by the invoking user (so subfolders are visible/editable in GUI)
mkdir -p "$MOUNT_POINT"
chown "$INV_UID:$INV_GID" "$MOUNT_BASE" || true

# ============================================================
# Step 4: Prepare removable-safe, high-throughput mount options
# ============================================================

# Explanation of options:
#  nofail                       : do not block boot if drive missing
#  x-systemd.device-timeout=5   : wait at most 5s for the device at boot
#  noatime,nodiratime           : reduce metadata writes (faster, less wear)
#  async                        : allow fully asynchronous I/O (performance)
#  commit=60                    : flush journal every 60s (trade safety for speed)
#  users                        : allow non-root user to mount/umount
FSTAB_OPTS="nofail,x-systemd.device-timeout=5,noatime,nodiratime,async,commit=60,users"

echo "Updating /etc/fstab with removable-safe, performance options..."
# Remove any existing entry for this UUID to avoid duplicates/conflicts
sed -i.bak "/UUID=${UUID//\//\\/}/d" /etc/fstab

# Add our new persistent mount entry (use UUID so re-plugging/port changes don't matter)
echo "UUID=$UUID  $MOUNT_POINT  ext4  $FSTAB_OPTS  0  2" | tee -a /etc/fstab >/dev/null

# ============================================================
# Step 5: Mount the filesystem now and set ownership
# ============================================================

# Create mountpoint (already created above) and mount immediately
mkdir -p "$MOUNT_POINT"
# If the device is present, this will mount; if not, it will just fail gracefully now
mount "$MOUNT_POINT" || true

# If mounted, ensure the root of the filesystem is owned by the invoking user
# (ext4 honors standard Unix ownership; this avoids needing sudo for writes)
if mountpoint -q "$MOUNT_POINT"; then
  chown "$INV_UID:$INV_GID" "$MOUNT_POINT" || true
fi

# ============================================================
# Step 6: Final checks and status
# ============================================================

echo
echo "=== Final checks ==="
# Show the active mount line (if mounted)
mount | grep -E "on ${MOUNT_POINT//\//\\/} " || echo "Not mounted yet; it will auto-mount next boot or run: sudo mount $MOUNT_POINT"
# Show space usage if mounted
df -h "$MOUNT_POINT" || true

# ============================================================
# Step 7 (Optional): Tweak queue depth for USB BOT throughput
# ============================================================

echo
echo "Optional: try raising queue_depth (if supported) to improve BOT throughput."
if [[ -f "/sys/block/$BASENAME/device/queue_depth" ]]; then
  echo -n "Current queue_depth: "
  cat "/sys/block/$BASENAME/device/queue_depth" || true
  read -rp "Set queue_depth=64 now? (yes/NO): " SET_QD
  SET_QD="${SET_QD,,}"
  if [[ "$SET_QD" == "yes" || "$SET_QD" == "y" ]]; then
    # Some USB bridges ignore this; failure here is non-fatal
    echo 64 > "/sys/block/$BASENAME/device/queue_depth" || echo "Could not set queue_depth; enclosure may not support it."
    echo -n "New queue_depth: "
    cat "/sys/block/$BASENAME/device/queue_depth" || true
  fi
else
  echo "queue_depth not exposed for this device."
fi

# ============================================================
# Step 8 (Optional): Enable write caching via hdparm
# ============================================================

echo
echo "Optional: enable device write caching (if supported by USB bridge)."
read -rp "Enable write caching now (hdparm -W1)? (yes/NO): " SET_WC
SET_WC="${SET_WC,,}"
if [[ "$SET_WC" == "yes" || "$SET_WC" == "y" ]]; then
  if command -v hdparm >/dev/null 2>&1; then
    # Many USB-SATA bridges ignore hdparm; this is best-effort
    hdparm -W1 "$DEV" || echo "hdparm failed or not supported over this USB bridge."
  else
    echo "hdparm not installed; skipping."
  fi
fi

# ============================================================
# Step 9: Tips & wrap-up
# ============================================================

echo
echo "All set! Your SSD is mounted (or will auto-mount) at: $MOUNT_POINT"
echo "It should appear in your file manager under /media/$INV_USER/$REAL_LABEL."
echo
echo "Speed tip for many small files (reduces per-file overhead):"
echo "  cd <SRC_DIR> && tar cf - . | pv | tar xf - -C \"$MOUNT_POINT\""
echo
echo "Note: With async,commit=60 a sudden power loss could lose up to ~60s of writes."
echo "      Use stable power when doing large transfers."
