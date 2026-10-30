"""Read public station sources; save dated results without asserting RF coverage."""
import concurrent.futures
import datetime
import json
import re
import urllib.request
import html
from pathlib import Path

URLS = {
    'aprs_org': 'https://www.aprs.org/',
    'findu_igates': 'http://www.findu.com/cgi-bin/igate.cgi',
    'brazil_PR2KW-1': 'https://aprs.fi/info/a/PR2KW-1',
    'panama_HP2DFA-6': 'https://aprs.fi/info/a/HP2DFA-6',
    'argentina_LU7FS-10': 'https://aprs.fi/info/a/LU7FS-10',
    'south_africa_directory': 'https://marc.org.za/marc_legacy/pages/freq.htm',
}

def get(item):
    key,url=item
    result={'key':key,'url':url,'checked_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat()}
    try:
        req=urllib.request.Request(url,headers={'User-Agent':'HB2A-APRS-reference-check/1.0'})
        with urllib.request.urlopen(req,timeout=20) as response:
            data=response.read(2_000_000).decode('utf-8',errors='replace')
            result.update(status=response.status,final_url=response.url,bytes=len(data))
        text=re.sub('<[^>]+>',' ',data)
        text=re.sub(r'\s+',' ',text)
        result['page_title']=re.findall(r'<title>(.*?)</title>',data,re.I|re.S)
        if key=='findu_igates':
            reports=[{'call':m[0],'report_age':m[1],'message_count':int(m[2]),'local_count':int(m[3])}
                     for m in re.findall(r'([A-Za-z0-9/-]+)\s+(\d+:\d+:\d+:\d+)\s+(\d+)\s+(\d+)',text)]
            result['reports']=reports
            result['report_count']=len(reports)
            rows=[]
            for tr in re.findall(r'<tr\b[^>]*>(.*?)</tr>',data,re.I|re.S):
                cells=[html.unescape(re.sub(r'\s+',' ',re.sub('<[^>]+>',' ',td))).strip() for td in re.findall(r'<t[dh]\b[^>]*>(.*?)</t[dh]>',tr,re.I|re.S)]
                if cells: rows.append(cells)
            result['table_headers']=rows[:1]
            result['page_intro']=text[:2000]
        if key=='south_africa_directory':
            start=text.find('APRS Digipeaters')
            if start>=0: text=text[start:]
        if not result.get('relevant_excerpts') and key not in ('findu_igates','aprs_org'):
            result['page_intro']=text[:500]
        result['relevant_excerpts']=[text[max(0,m.start()-100):m.end()+200] for m in list(re.finditer(r'144\.\d+|145\.\d+|directly heard|last heard|ZS0PMB|live IGate list',text,re.I))[:20]]
        if key=='aprs_org':
            result['igate_links']=[x for x in re.findall(r'''href=["']([^"']+)''',data,re.I) if 'igate' in x.lower()]
        # A successful HTTP response is not evidence of RF reception.
        result['rf_reception_confirmed']=False
        result['browser_challenge']='Verifying your browser' in text
    except Exception as exc:
        result['error']=str(exc)
    return result

if __name__=='__main__':
    with concurrent.futures.ThreadPoolExecutor(max_workers=6) as pool:
        results=list(pool.map(get,URLS.items()))
    Path(__file__).with_name('station_source_checks.json').write_text(json.dumps(results,indent=2,ensure_ascii=False),encoding='utf-8')
    for result in results:
        print(result['key'],result.get('status',result.get('error')),result.get('igate_links',''))
