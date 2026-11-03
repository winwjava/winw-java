import base64,csv,gzip,json,struct,argparse
from pathlib import Path
parser=argparse.ArgumentParser();parser.add_argument('surfaces',type=Path);parser.add_argument('occupancy',type=Path);parser.add_argument('--inline',type=Path);args=parser.parse_args()
occ=json.loads((args.occupancy/'occupancy-data.json').read_text(encoding='utf-8'))
if 'thumbnail' not in occ['frames'][0]:
 thumbs=json.loads((args.occupancy/'thumbnails.json').read_text(encoding='utf-8-sig'))
 for f,t in zip(occ['frames'],thumbs):f['thumbnail']=t
quality={int(row['frame']):row for row in csv.DictReader((args.surfaces/'quality.csv').open())}
frames=[]
for old in occ['frames']:
 i=old['index'];path=args.surfaces/'surfaces'/f'{i:03d}.surf'
 if not path.exists():continue
 raw=path.read_bytes();q=quality[i]
 frames.append({'index':i,'thumbnail':old['thumbnail'],'voxels':old['voxels'],'mesh':base64.b64encode(gzip.compress(raw,compresslevel=6,mtime=0)).decode(),'trusted':int(q['trusted_pixels']),'ms':float(q['ms'])})
data={'frames':frames}
source=Path(__file__).with_name('surface-viewer.fragment.html').read_text(encoding='utf-8')
css='''<style>:root{color-scheme:light dark;--foreground:light-dark(#182633,#edf1f5);--background:light-dark(#ffffff,#17212a);--muted:light-dark(#edf1f4,#26343e);--border:light-dark(#9caab5,#485d6a);--blue:#329b95;--viz-series-1:#329b95}body{font:16px system-ui;margin:24px;background:var(--background);color:var(--foreground)}h3{font-weight:500}.viz-controls{display:flex;gap:16px;align-items:center;flex-wrap:wrap;margin-bottom:14px}.form-label{display:inline-block}.form-range{width:100%}.btn,.form-select{font:inherit;padding:6px 12px}.text-small{font-size:12px;margin-top:10px}</style>'''
fragment=source.replace('__SURFACE_DATA__',json.dumps(data,separators=(',',':')))
page='<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>双目三维环境 · 改进前后对照</title>'+css+fragment+'</html>'
(args.surfaces/'index.html').write_text(page,encoding='utf-8')
# Keep the already-open result URL useful, preserving the former viewer for comparison.
old=args.occupancy/'index.html';backup=args.occupancy/'voxel-viewer.html'
if not backup.exists() and old.exists():backup.write_bytes(old.read_bytes())
old.write_text(page,encoding='utf-8')
if args.inline:
 selected={0,12,23,55,119};data['frames']=[f for f in frames if f['index'] in selected]
 while True:
  fragment=source.replace('__SURFACE_DATA__',json.dumps(data,separators=(',',':')))
  if len(fragment.encode())<1000000:break
  removable=next((i for i,f in enumerate(data['frames']) if f['index']!=12),None)
  if removable is None:raise ValueError('Inline surface exceeds 1 MB')
  del data['frames'][removable]
 args.inline.parent.mkdir(parents=True,exist_ok=True);args.inline.write_text(fragment,encoding='utf-8')
 print('Inline bytes:',len(fragment.encode()),'frames:',[f['index'] for f in data['frames']])
print('Full viewer bytes:',len(page.encode()),'frames:',len(frames))
