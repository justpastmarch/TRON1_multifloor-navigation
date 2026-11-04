"""New-site map library. Never writes production floor maps."""
import base64
import json
import math
import os
from pathlib import Path
import re
import time
import uuid

import yaml

ID = re.compile(r'^[a-zA-Z0-9_-]{1,100}$')

def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + '.tmp')
    with temporary.open('w') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)

class MapLibrary:
    def __init__(self, root):
        self.root = Path(root).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)

    def create(self, name):
        name = str(name).strip()
        if not name or len(name) > 100:
            raise ValueError('장소 이름을 1~100자로 입력하세요.')
        identifier = time.strftime('%Y%m%d_%H%M%S_') + uuid.uuid4().hex[:8]
        path = self.root / identifier
        path.mkdir()
        atomic_json(path / 'session.json', dict(id=identifier, name=name, created=time.time()))
        return identifier, path

    def path(self, identifier):
        if not isinstance(identifier, str) or not ID.fullmatch(identifier):
            raise ValueError('잘못된 지도 ID입니다.')
        path = self.root / identifier
        if not path.is_dir() or path.is_symlink():
            raise ValueError('지도 기록을 찾을 수 없습니다.')
        return path

    def list(self):
        rows = []
        for path in sorted(self.root.iterdir(), reverse=True):
            if path.is_dir() and not path.is_symlink() and (path/'session.json').is_file():
                row = json.loads((path/'session.json').read_text())
                row['maps'] = []
                for meta in sorted(path.glob('*/result.json')):
                    result = json.loads(meta.read_text())
                    result['key'] = meta.parent.name
                    row['maps'].append(result)
                row['bag_ready'] = (path/'input.bag').is_file()
                row['comparison'] = json.loads((path/'comparison.json').read_text()) if (path/'comparison.json').is_file() else None
                rows.append(row)
        return rows

    def result_path(self, identifier, key):
        root = self.path(identifier)
        if not isinstance(key, str) or not ID.fullmatch(key):
            raise ValueError('잘못된 지도 버전입니다.')
        target = root/key
        if not target.is_dir() or target.is_symlink():
            raise ValueError('저장된 지도를 찾을 수 없습니다.')
        return target

    def file(self, identifier, key, name):
        if name not in ('map.pgm', 'map.yaml', 'graph.posegraph', 'graph.data'):
            raise ValueError('지원하지 않는 파일입니다.')
        path = self.result_path(identifier,key)/name
        if not path.is_file() or path.is_symlink():
            raise ValueError('해당 파일이 없습니다.')
        return path.read_bytes()

    def preview(self, identifier, key):
        from mission_manager.console_map import encode_map
        from nav_msgs.msg import OccupancyGrid
        from PIL import Image
        path = self.result_path(identifier,key)
        meta = yaml.safe_load((path/'map.yaml').read_text())
        image_path = (path/meta['image']).resolve()
        if image_path != (path/'map.pgm').resolve():
            raise ValueError('지도 이미지 경로가 올바르지 않습니다.')
        image = Image.open(image_path).convert('L')
        message = OccupancyGrid()
        message.header.frame_id='map'
        message.info.width, message.info.height=image.size
        message.info.resolution=float(meta['resolution'])
        x,y,yaw=meta['origin']
        message.info.origin.position.x=x; message.info.origin.position.y=y
        message.info.origin.orientation.z=math.sin(yaw/2)
        message.info.origin.orientation.w=math.cos(yaw/2)
        import numpy as np
        pixels=np.array(image)[::-1].astype(float)
        probability=(pixels if meta.get('negate') else 255-pixels)/255
        grid=np.full(pixels.shape,-1,dtype=int)
        grid[probability>meta['occupied_thresh']]=100
        grid[probability<meta['free_thresh']]=0
        message.data=grid.ravel().tolist()
        return encode_map(message)


def map_metadata(directory):
    """Validate and summarize a map_server pair after it has actually been saved."""
    from PIL import Image
    import numpy as np
    path=Path(directory)
    metadata=yaml.safe_load((path/'map.yaml').read_text())
    if (path/metadata['image']).resolve() != (path/'map.pgm').resolve():
        raise ValueError('저장된 YAML과 PGM이 일치하지 않습니다.')
    resolution=float(metadata['resolution'])
    if not math.isfinite(resolution) or resolution<=0:
        raise ValueError('지도 해상도가 잘못되었습니다.')
    image=Image.open(path/'map.pgm').convert('L');pixels=np.asarray(image)
    result=dict(width=image.width,height=image.height,resolution=resolution,
                known_cells=int((pixels!=205).sum()),occupied_cells=int((pixels<50).sum()),
                origin=metadata['origin'],saved=time.time())
    return result
