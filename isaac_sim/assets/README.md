# Go1 stage

`go1_comp.usd` is the tested Isaac Sim 5.1 stage used by this integration. It
composes NVIDIA's Isaac Sim Go1 sensor asset and adds the simulation settings
expected by `../scripts/go1_isaac_bridge.py`.

The referenced NVIDIA asset is not redistributed here. It must be available
through the Isaac Sim asset browser/cache. NVIDIA's terms apply to that asset.
The expected robot paths are `/World/go1` and `/World/go1/trunk`.
