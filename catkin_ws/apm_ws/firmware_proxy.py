#!/usr/bin/env python3

import bottle
import dns.resolver
import random
import requests
import urllib.parse

FIRMWARE_SITE = 'firmware.ardupilot.org'
DNS_CACHE = dns.resolver.Cache()

@bottle.get('/')
def root():
    return forward('')

@bottle.get('/<path:path>')
def forward(path):
    answer = DNS_CACHE.get(FIRMWARE_SITE)
    if answer is None:
        answer = dns.resolver.query(FIRMWARE_SITE)
        DNS_CACHE.put(FIRMWARE_SITE, answer)
    real_host = random.choice(answer).to_text()
    print(real_host)
    response = requests.get('http://{}/{}'.format(real_host, path),
            headers={'Host': FIRMWARE_SITE}, allow_redirects=False)
    while 'location' in response.headers:
        location = response.headers['location']
        parsed = urllib.parse.urlparse(location)
        if 'firmware.ardupilot.org' in parsed.netloc:
            session = urllib.parse.parse_qs(parsed.query)['X-OpenDNS-Session']
            response = requests.get('http://{}/{}'.format(real_host, path),
                    headers={'Host': FIRMWARE_SITE}, allow_redirects=False,
                    params={'X-OpenDNS-Session': session})
        else:
            response = requests.get(location, allow_redirects=False)
    bottle.response.status = response.status_code
    bottle.response.content_type = response.headers['content-type']
    return response.content

if __name__ == '__main__':
    bottle.run(host='localhost', port=80)
