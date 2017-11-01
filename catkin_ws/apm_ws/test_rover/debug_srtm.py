#!/usr/bin/env python
from MAVProxy.modules.mavproxy_map.srtm import SRTMDownloader
#DEBUG ONLY
if __name__ == '__main__':
    downloader = SRTMDownloader(debug=True)
    downloader.loadFileList()
    import time
    start = time.time()
    while time.time() - start < 30:
        tile = downloader.getTile(-36, 149)
        if tile:
            print tile.getAltitudeFromLatLon(-35.282, 149.1287)
            break
        time.sleep(0.2)
