#!/usr/bin/env python3

import bottle
import dns.resolver
import random
import requests

FIRMWARE_SITE = 'firmware.ardupilot.org'
DNS_CACHE = dns.resolver.Cache()

@bottle.get('/<path:path>')
def forward(path):
    answer = DNS_CACHE.get(FIRMWARE_SITE)
    if answer is None:
        answer = dns.resolver.query(FIRMWARE_SITE)
        DNS_CACHE.put(FIRMWARE_SITE, answer)
    real_hsot = random.choice(answer)
    response = requests.get('http://{}/{}'.format(real_host),
            headers={'Host': FIRMWARE_SITE})
    return response.body

if __name__ == '__main__':
    bottle.run(host=FIRMWARE_SITE, port=80)
