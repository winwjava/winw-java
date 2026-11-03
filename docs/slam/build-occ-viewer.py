"""Build offline 3D voxel viewer; no third-party Python packages or network access."""
import argparse
import json
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('output', type=Path)
parser.add_argument('--inline', type=Path)
args = parser.parse_args()
data = json.loads((args.output / 'occupancy-data.json').read_text(encoding='utf-8'))
template = Path(__file__).with_name('occ-viewer.fragment.html').read_text(encoding='utf-8')
css = '''<style>:root{color-scheme:light dark;--foreground:light-dark(#182633,#edf1f5);--background:light-dark(#ffffff,#17212a);--muted:light-dark(#edf1f4,#26343e);--muted-foreground:light-dark(#52616d,#bbc8d2);--border:light-dark(#9caab5,#485d6a);--blue:#329b95;--viz-series-1:#329b95}body{font:16px system-ui;margin:24px;background:var(--background);color:var(--foreground)}h3{font-weight:500}.viz-controls{display:flex;gap:16px;align-items:center;flex-wrap:wrap;margin-bottom:14px}.form-label{display:inline-block}.form-range{width:100%}.btn,.form-select{font:inherit;padding:6px 12px}.text-small{font-size:12px;margin-top:10px}</style>'''
fragment = template.replace('__OCC_DATA__', json.dumps(data, separators=(',', ':')))
page = '<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>实际三维 OCC 查看器</title>' + css + fragment + '</html>'
(args.output / 'index.html').write_text(page, encoding='utf-8')
if args.inline:
    selected = {0, 6, 12, 18, 23, 30, 45, 55, 62, 75, 90, 119}
    data['frames'] = [f for f in data['frames'] if f['index'] in selected]
    fragment = template.replace('__OCC_DATA__', json.dumps(data, separators=(',', ':')))
    if len(fragment.encode('utf-8')) > 1_000_000:
        raise ValueError('Inline preview exceeds 1 MB')
    args.inline.parent.mkdir(parents=True, exist_ok=True)
    args.inline.write_text(fragment, encoding='utf-8')
print(args.output.resolve() / 'index.html')
