"""Offline 1-degree APRS candidate grid and random-access RLE. See README."""
import hashlib
import json
import math
import struct
import zlib
from pathlib import Path
import numpy as np
from shapely.geometry import shape, box
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.patches import Patch

ROOT=Path(__file__).resolve().parent
UNKNOWN=0x8000
source=ROOT/'countries_50m.geojson'
features=json.loads(source.read_text(encoding='utf-8'))['features']
rules=json.loads((ROOT/'frequency_rules.json').read_text(encoding='utf-8'))
frequencies=rules['frequencies_hz']
def mask_for(hz):
    return sum(1<<frequencies.index(f) for f in set(hz))
specific={c:mask_for(g['frequencies_hz']) for g in rules['groups'] for c in g['countries']}
def merge(a,b):
    combined=int(a)|int(b)
    known=combined&0xff
    return known if known else combined
assert merge(UNKNOWN,1)==merge(1,UNKNOWN)==1
assert merge(0,1)==merge(1,0)==1
assert merge(1,2)==3 and merge(3,UNKNOWN)==3
assert merge(UNKNOWN,0)==UNKNOWN
grid=np.zeros((180,360),dtype=np.uint16)
assigned={}; excluded=[]
for feature in features:
    props=feature['properties']; country=props['ADM0_A3']
    europe=props['CONTINENT']=='Europe' and country not in specific
    mask=specific.get(country,mask_for([rules['europe']['frequency_hz']]) if europe else UNKNOWN)
    assigned[country]={'name':props['ADMIN'],'mask':mask,'basis':'regional' if europe else 'explicit' if country in specific else 'unconfirmed'}
    geom=shape(feature['geometry'])
    for part in (list(geom.geoms) if geom.geom_type=='MultiPolygon' else [geom]):
        part_mask=mask
        if europe and country not in rules['europe']['whole_country_exceptions']:
            p=part.representative_point(); w,s,e,n=rules['europe']['component_bounds']
            if not (w<=p.x<=e and s<=p.y<=n):
                part_mask=UNKNOWN
                excluded.append({'country':country,'representative_point':[p.x,p.y]})
        west,south,east,north=part.bounds
        for lat in range(max(-90,math.floor(south)),min(90,math.ceil(north))):
            for lon in range(max(-180,math.floor(west)),min(180,math.ceil(east))):
                if part.intersection(box(lon,lat,lon+1,lat+1)).area>1e-10:
                    r,c=lat+90,lon+180
                    grid[r,c]=merge(grid[r,c],part_mask)
def lookup(lat,lon):
    if not math.isfinite(lat) or not math.isfinite(lon) or not -90<=lat<=90 or not -180<=lon<=180:
        raise ValueError('Invalid coordinates')
    return int(grid[min(179,math.floor(lat+90)),math.floor((lon+180)%360)])
samples=[(35.68,139.69,[144660000]),(39.9,116.4,[144640000]),(40.71,-74,[144390000]),
         (51.5,-.12,[144800000]),(-33.86,151.2,[145175000]),(-41.3,174.8,[144575000]),
         (-23.55,-46.63,[145570000]),(-34.6,-58.4,[144390000,144930000]),
         (-29.6,30.4,[144800000]),(-34.9,-56.2,[144930000]),(9.26,-79.82,[144930000]),
         (13.75,100.5,[144390000]),(0,-140,[])]
for lat,lon,hz in samples:
    assert lookup(lat,lon)==mask_for(hz),(lat,lon,lookup(lat,lon),hz)
assert lookup(0,180)==lookup(0,-180)
assert lookup(4.9,-52.3)==UNKNOWN
assert not np.any((grid&UNKNOWN!=0)&(grid&0xff!=0))
palette=sorted(int(x) for x in np.unique(grid))
assert len(palette)<=128
ids=np.searchsorted(palette,grid).astype(np.uint8)
words=[]; offsets=[0]
for row in ids:
    start=0
    for col in range(1,361):
        if col==360 or row[col]!=row[start]:
            words.append((int(row[start])<<9)|col)
            start=col
    offsets.append(len(words))
assert len(words)<65536
def decode(r,c):
    lo,hi=offsets[r],offsets[r+1]
    while lo<hi:
        mid=(lo+hi)//2
        if (words[mid]&511)<=c: lo=mid+1
        else: hi=mid
    return palette[words[lo]>>9]
for r in range(180):
    for c in range(360):
        assert decode(r,c)==int(grid[r,c]),(r,c)
raw_ids=ids.tobytes()
(ROOT/'aprs_1deg.bin').write_bytes(raw_ids)
(ROOT/'aprs_1deg_masks.bin').write_bytes(grid.astype('<u2').tobytes())
packed=struct.pack('<181H',*offsets)+struct.pack('<%dH'%len(palette),*palette)+struct.pack('<%dH'%len(words),*words)
(ROOT/'aprs_1deg_rle.bin').write_bytes(packed)
table_bytes=len(packed)+4*len(frequencies)
meta={'format_version':2,'resolution_degrees':1,'rows':180,'columns':360,
      'order':'south-to-north, west-to-east, row-major',
      'frequency_bits_hz':{str(i):hz for i,hz in enumerate(frequencies)},
      'special_masks':{'0':'no mapped candidate, NOT no receivers','32768':'frequency unconfirmed, NOT no receivers','16384':'invalid position (lookup only)'},
      'palette_masks':palette,'country_assignments':assigned,'overseas_components_not_inherited':excluded,
      'compression':{'original_uint8_bytes':64800,'reference_mask_bytes':129600,
       'palette_uint8_bytes':len(raw_ids),'bitpacked_palette_bytes':math.ceil(64800*math.ceil(math.log2(len(palette)))/8)+2*len(palette),
       'zlib_palette_bytes_without_decoder':len(zlib.compress(raw_ids,9))+2*len(palette),
       'row_rle_words':len(words),'max_runs_per_row':max(b-a for a,b in zip(offsets,offsets[1:])),
       'row_rle_bytes':len(packed),'total_c_table_bytes_including_frequencies':table_bytes,
       'reduction_vs_original_percent':round(100*(1-table_bytes/64800),2)},
      'geometry_source':'https://github.com/nvkelso/natural-earth-vector/blob/master/geojson/ne_50m_admin_0_countries.geojson',
      'geometry_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
      'rules_sha256':hashlib.sha256((ROOT/'frequency_rules.json').read_bytes()).hexdigest(),
      'mask_cell_counts':{str(int(k)):int(v) for k,v in zip(*np.unique(grid,return_counts=True))}}
(ROOT/'aprs_1deg.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2),encoding='utf-8')
def c_array(name,kind,values,width=16):
    return 'static const '+kind+' '+name+'['+str(len(values))+'] = {\n'+''.join('  '+','.join(str(x) for x in values[i:i+width])+',\n' for i in range(0,len(values),width))+'};\n'
header='''/* Generated reference v2. Read README.md. No automatic transmit policy. */
#ifndef HB2A_APRS_REFERENCE_GRID_H
#define HB2A_APRS_REFERENCE_GRID_H
#include <stdint.h>
#define APRS_GRID_UNKNOWN 0x8000U
#define APRS_GRID_INVALID 0x4000U
#define APRS_GRID_FREQUENCY_MASK 0x00FFU
'''
header+=c_array('aprs_grid_frequencies_hz','uint32_t',frequencies)
header+=c_array('aprs_grid_row_offsets','uint16_t',offsets)
header+=c_array('aprs_grid_palette','uint16_t',palette)
header+=c_array('aprs_grid_runs','uint16_t',words)
header+='''
/* All frequency candidates, or a special mask. No heap/decompression. */
static inline uint16_t aprs_grid_lookup_e7(int32_t latitude_e7,
                                         int32_t longitude_e7, uint8_t valid)
{
  uint32_t row, col;
  uint16_t lo, hi;
  int64_t lon;
  if (!valid || latitude_e7 < -900000000L || latitude_e7 > 900000000L ||
      longitude_e7 < -1800000000L || longitude_e7 > 1800000000L)
    return APRS_GRID_INVALID;
  row = (uint32_t)(((int64_t)latitude_e7 + 900000000LL) / 10000000LL);
  if (row == 180U) row = 179U;
  lon = (int64_t)longitude_e7 + 1800000000LL;
  if (lon == 3600000000LL) lon = 0;
  col = (uint32_t)(lon / 10000000LL);
  lo = aprs_grid_row_offsets[row];
  hi = aprs_grid_row_offsets[row + 1U];
  while (lo < hi) {
    uint16_t mid = (uint16_t)(lo + (hi - lo) / 2U);
    if ((aprs_grid_runs[mid] & 511U) <= col) lo = (uint16_t)(mid + 1U);
    else hi = mid;
  }
  return aprs_grid_palette[aprs_grid_runs[lo] >> 9U];
}
/* 0 means no unique candidate, never a valid radio frequency command. */
static inline uint32_t aprs_grid_single_frequency_hz(uint16_t mask)
{
  unsigned i;
  if ((mask & ~APRS_GRID_FREQUENCY_MASK) || !mask || (mask & (mask - 1U))) return 0;
  for (i = 0; i < 8U; ++i)
    if (mask == (uint16_t)(1U << i)) return aprs_grid_frequencies_hz[i];
  return 0;
}
#endif
'''
(ROOT/'aprs_1deg.h').write_text(header,encoding='utf-8')
colors=['#e8edf2','#e69f00','#56b4e9','#009e73','#cc79a7','#d55e00','#0072b2','#805030','#aa4499','#b9bdc2','#272b31']
display=np.zeros(grid.shape,dtype=np.uint8)
for i in range(8): display[grid==(1<<i)]=i+1
display[grid==UNKNOWN]=9
display[(grid>0)&(grid!=UNKNOWN)&((grid&(grid-1))!=0)]=10
fig,ax=plt.subplots(figsize=(16,8))
ax.imshow(display,origin='lower',extent=(-180,180,-90,90),interpolation='nearest',cmap=ListedColormap(colors),norm=BoundaryNorm(np.arange(-.5,11.5),11))
for f in features:
    g=shape(f['geometry'])
    for p in (list(g.geoms) if g.geom_type=='MultiPolygon' else [g]):
        x,y=p.exterior.xy; ax.plot(x,y,color='#555555',lw=.2)
ax.set(xticks=range(-180,181,30),yticks=range(-90,91,30),xlabel='Longitude (degrees)',ylabel='Latitude (degrees)',title='APRS 1200 bps frequency candidates - 1 degree grid v2 (not receiver coverage)')
ax.grid(alpha=.2)
labels=['No mapped candidate']+[f'{hz/1e6:.3f} MHz' for hz in frequencies]+['Frequency unconfirmed','Multiple known candidates']
ax.legend(handles=[Patch(color=c,label=l) for c,l in zip(colors,labels)],loc='lower center',bbox_to_anchor=(.5,-.25),ncol=4,frameon=False)
fig.tight_layout(); fig.savefig(ROOT/'aprs_1deg.png',dpi=160,bbox_inches='tight')
print(json.dumps(meta['compression'],indent=2))
print('PASS: known/unknown precedence, 13 geographic samples, overseas exclusion, all 64,800 cells RLE round-trip')
