# NeuROAM project page

Static GitHub Pages site for [NeuROAM](https://neufieldrobotics.github.io/NeuROAM/).

Served from this `webpage` branch. Rebuild derived assets with:

```bash
python3 tools/build_site_data.py
python3 tools/process_cad.py --step /path/to/NEUROAM_PAYLOAD.STEP --out static/models/payload.glb
```

Preview locally: `python3 -m http.server 8765`
