# Steps to debayer
1. Run the orginal bag
2. Launch the script with `python3 debayer_node.py`
3. The script will read topics `/cam_sync/cam0/image_raw/compressed` and `/cam_sync/cam1/image_raw/compressed` and publish uncompressed and debayered to `/cam_sync/cam0/image_raw/debayered` and `/cam_sync/cam1/image_raw/debayered`
