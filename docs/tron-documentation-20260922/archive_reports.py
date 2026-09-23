from pathlib import Path
from bs4 import BeautifulSoup
import base64,hashlib,json,os,re,shutil,subprocess,unicodedata
from urllib.parse import urlsplit,unquote,quote

OUT=Path(__file__).resolve().parent
ROOT=OUT.parent
found=subprocess.check_output(['rg','--files','--hidden','--no-ignore','-g','*.html','-g','*.htm','-g','!**/.git/**','-g','!sources/**','-g','!tron-documentation-20260922/**'],cwd=ROOT,text=True).splitlines()
SOURCES=sorted((ROOT/x).resolve() for x in found)
MAP={p:OUT/'documents'/p.relative_to(ROOT).with_suffix('.md') for p in SOURCES}
(OUT/'assets').mkdir(exist_ok=True)
manifest={'scope':'Generated HTML reports under the current project; sources/ and .git excluded.','format':'Markdown','html_count':len(SOURCES),'documents':[],'assets':[],'external_local_references':[],'missing_references':[]}
assets={}

def digest(data):return hashlib.sha256(data).hexdigest()
def add_asset(data,suffix,source):
 h=digest(data);suffix=suffix.lower() if re.fullmatch(r'\.[A-Za-z0-9]{1,10}',suffix) else '.bin'
 dest=OUT/'assets'/(h[:20]+suffix)
 if not dest.exists():dest.write_bytes(data)
 assets[str(dest.relative_to(OUT))]={'path':str(dest.relative_to(OUT)),'bytes':len(data),'sha256':h,'source':source}
 return dest

def local_url(dest,md):return quote(os.path.relpath(dest,md.parent),safe='/._-')

def resolve_url(url,source,md,media=False):
 if not url:return url
 if url.startswith('data:'):
  m=re.match(r'data:([^;,]+)(;base64)?,(.*)',url,re.S)
  if not m:return url
  typ,b64,payload=m.groups();data=base64.b64decode(payload) if b64 else unquote(payload).encode()
  ext={'image/png':'.png','image/jpeg':'.jpg','image/svg+xml':'.svg','image/webp':'.webp'}.get(typ,'.bin')
  return local_url(add_asset(data,ext,source.name+':embedded '+typ),md)
 parts=urlsplit(url)
 if parts.scheme not in ('','file'):return url
 if not parts.path:return url
 raw=unquote(parts.path)
 path=Path(raw) if raw.startswith('/') else source.parent/raw
 path=path.resolve()
 if path in MAP:
  return local_url(MAP[path],md)+(('#'+parts.fragment) if parts.fragment else '')
 if media or path.suffix.lower() in ('.png','.jpg','.jpeg','.webp','.gif','.svg','.mp4','.webm','.mp3','.wav'):
  if path.is_file():
   return local_url(add_asset(path.read_bytes(),path.suffix,str(path)),md)+(('#'+parts.fragment) if parts.fragment else '')
 stripped=re.sub(r':\d+$','',str(path))
 exists=Path(stripped).exists()
 entry={'document':str(md.relative_to(OUT)),'target':str(path),'exists':exists}
 manifest['external_local_references'].append(entry)
 if not exists:manifest['missing_references'].append(entry)
 return quote(str(path),safe='/:._-')+(('#'+parts.fragment) if parts.fragment else '')

def norm(x):return ''.join(ch for ch in unicodedata.normalize('NFKC',x) if ch.isalnum()).casefold()

for source in SOURCES:
 md=MAP[source];md.parent.mkdir(parents=True,exist_ok=True)
 raw=source.read_bytes();soup=BeautifulSoup(raw,'html.parser')
 title=soup.title.get_text(' ',strip=True) if soup.title else source.stem
 counts={tag:len(soup.find_all(tag)) for tag in ['table','img','svg','video','script','nav']}
 body=soup.body or soup
 # Static content is fully retained; scripts only provide navigation/filter/playback UI.
 for x in list(body.find_all(['script','style','nav','form','input','select'])):x.decompose()
 for n,svg in enumerate(list(body.find_all('svg')),1):
  if not svg.get('xmlns'):svg['xmlns']='http://www.w3.org/2000/svg'
  labels=' '.join(x.get_text(' ',strip=True) for x in svg.find_all('text'))
  dest=add_asset(str(svg).encode(),'.svg',str(source)+f':svg{n}')
  img=soup.new_tag('img',src=local_url(dest,md),alt='도표 '+str(n))
  svg.replace_with(img)
  if labels:
   caption=soup.new_tag('p');caption.string='도표의 텍스트: '+labels;img.insert_after(caption)
 for n,video in enumerate(list(body.find_all(['video','audio'])),1):
  url=video.get('src') or (video.find('source').get('src') if video.find('source') else '')
  wrapper=soup.new_tag('div')
  if video.get('poster'):
   wrapper.append(soup.new_tag('img',src=resolve_url(video['poster'],source,md,True),alt=f'영상 {n} 미리보기'))
  a=soup.new_tag('a',href=resolve_url(url,source,md,True));a.string=f'영상 {n} 재생 / 원본 파일'
  p=soup.new_tag('p');p.append(a);wrapper.append(p);video.replace_with(wrapper)
 for button in body.find_all('button'):
  onclick=button.get('onclick','')
  t=re.search(r'jump\([^,]+,\s*([\d.]+)',onclick)
  if t:button.append(' (재생 위치 '+t.group(1)+'초)')
  button.name='span';button.attrs={}
 # Preserve custom anchors dropped by GFM's heading renderer.
 for node in list(body.find_all(id=True)):
  ident=node.get('id')
  anchor=soup.new_tag('span',id=ident);node.insert_before(anchor);del node['id']
 for node in body.find_all(['a','img','source']):
  attr='href' if node.name=='a' else 'src'
  if not node.get(attr):continue
  value=node[attr]
  # Already archived generated SVG/poster/video references.
  if value.startswith('../../') and '/assets/' in value:continue
  if value.startswith('../') and 'assets/' in value:continue
  node[attr]=resolve_url(value,source,md,node.name in ('img','source'))
 for p in body.find_all(['p','div','section','article','main']):
  if p.get('style') and 'display:none' in p['style'].replace(' ',''):del p['style']
 # Flatten presentation containers so figure-only HTML handling cannot discard captions/details.
 for node in list(body.find_all('summary')):node.name='p'
 for node in list(body.find_all(['div','section','article','main','header','footer','figure','details','aside'])):node.unwrap()
 for node in list(body.find_all('span')):
  if not node.get('id'):node.unwrap()
 normalized_html=str(body)
 result=subprocess.run(['pandoc','-f','html','-t','gfm','--wrap=none'],input=normalized_html,text=True,capture_output=True,check=True)
 # Native Markdown tables where possible; Pandoc preserves complex line-break tables as raw HTML.
 text=result.stdout
 header=f'# {title}\n\n> 보관일: 2026-09-22. 원본 HTML을 당시 내용 그대로 옮긴 기록입니다. 현재 적용 상태는 [최신 적용 안내]({local_url(OUT/"APPLICATION_GUIDE.md",md)})를 확인합니다.\n\n원본: [{source.relative_to(ROOT)}]({quote(str(source),safe="/:._-")}) · SHA-256: `{digest(raw)}`\n\n---\n\n'
 md.write_text(header+text)
 # Round-trip to plain text and test every paragraph/list-cell/heading text block.
 plain=subprocess.run(['pandoc','-f','gfm','-t','plain','--wrap=none'],input=header+text,text=True,capture_output=True,check=True).stdout
 full=norm(plain)
 missed=[];checked=0
 for node in body.find_all(['p','li','th','td','h1','h2','h3','h4','h5','h6','summary']):
  value=node.get_text(' ',strip=True);n=norm(value)
  if len(n)<2:continue
  checked+=1
  if n not in full:missed.append(value[:240])
 manifest['documents'].append({'source':str(source.relative_to(ROOT)),'source_sha256':digest(raw),'source_bytes':len(raw),'title':title,'markdown':str(md.relative_to(OUT)),'markdown_sha256':digest(md.read_bytes()),'source_elements':counts,'text_blocks_checked':checked,'text_blocks_not_found':missed,'status':'CONVERTED_CHECK_REQUIRED' if missed else 'CONVERTED_VERIFIED'})
manifest['assets']=list(assets.values())
manifest['source_html_unchanged']=all(digest((ROOT/d['source']).read_bytes())==d['source_sha256'] for d in manifest['documents'])
(OUT/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
print(json.dumps({'html_count':len(SOURCES),'assets':len(assets),'asset_bytes':sum(x['bytes'] for x in assets.values()),'text_blocks_checked':sum(x['text_blocks_checked'] for x in manifest['documents']),'text_blocks_not_found':{x['source']:x['text_blocks_not_found'] for x in manifest['documents'] if x['text_blocks_not_found']},'missing_references':manifest['missing_references'],'source_html_unchanged':manifest['source_html_unchanged']},ensure_ascii=False,indent=2))
