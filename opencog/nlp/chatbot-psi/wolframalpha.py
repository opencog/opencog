import urllib2
import xml.etree.ElementTree as ET

def call_wolframalpha(qq):
    # Enter your own key here!
    appid = ''

    # Avoid HTTP Error 400: Bad Request
    # query = qq.name.replace(' ', '+')
    # XXX
    query = qq.replace(' ', '+')

    url_1 = 'http://api.wolframalpha.com/v2/query?appid='
    url_2 = '&input='
    url_3 = '&format=plaintext'
    full_url = url_1 + appid + url_2 + query + url_3

    response = ET.fromstring(urllib2.urlopen(full_url).read())
    result = ''

    # List of pod titles it's currently checking:
    # - Result
    # - Definition
    # - Basic definition
    # Usually the answer is in 'Result', if not, usually one of
    # them has the answer
    # TODO: Expand to cover more if needed
    for pod in response.iter('pod'):
        title = pod.get('title')
        if title == 'Result' or title == 'Definition' or title == 'Basic definition':
            result = pod.find('subpod').find('plaintext').text

    print result.replace('|', ',')

# XXX For testing only, to be removed
import sys
call_wolframalpha(' '.join(sys.argv[1:]))
