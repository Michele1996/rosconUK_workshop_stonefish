#!/usr/bin/env python3
"""Create a Stonefish-compatible 16-bit bathymetry height map and seabed colour texture.

Run from the package root. Accepts a texture photograph/artwork (--texture) and,
optionally, a separately supplied grayscale measured or synthetic bathymetry
(--height-source). A colour image by itself cannot recover true geometry; when
no height-source is given, the generated relief is artificial and reproducible.

Outputs data/photogrammetry/terrain_height.png (16-bit grayscale),
and data/photogrammetry/textures/seabed.jpg (RGB).
"""
from pathlib import Path
import argparse
import numpy as np
from PIL import Image, ImageFilter

def resample(fp, side):
    return Image.open(fp).convert("RGB").resize((side,side),Image.Resampling.LANCZOS)

def smooth(a, sigma):
    from PIL import Image
    im=Image.fromarray(np.uint8(np.clip(a*255,0,255)),"L")
    return np.asarray(im.filter(ImageFilter.GaussianBlur(radius=sigma)),dtype=np.float32)/255.0

def generate(texture=None,height_source=None,side=512,seed=2026,amplitude=1.2,out=None):
    side=int(side)
    if side<64 or side>2048:raise ValueError("--size must be between 64 and 2048")
    out=Path(out) if out else Path(__file__).resolve().parent.parent/'data'/'photogrammetry'
    out.mkdir(parents=True,exist_ok=True); (out/'textures').mkdir(parents=True,exist_ok=True)
    rng=np.random.default_rng(seed)
    yy,xx=np.mgrid[:side,:side].astype('float32'); xx=(xx/(side-1)-.5)*2; yy=(yy/(side-1)-.5)*2
    noise=np.zeros((side,side),dtype=np.float32)
    for grid,weight in [(12,.48),(28,.20),(62,.09),(160,.03)]:
        small=np.uint8(rng.integers(0,256,(grid,grid)))
        n=np.asarray(Image.fromarray(small,'L').resize((side,side),Image.Resampling.BICUBIC),dtype=np.float32)/255
        noise+=weight*(n-.5)
    ridges=.15*np.sin(xx*11+yy*6+3*np.sin(yy*3))*np.exp(-(xx*xx+yy*yy)*.8)
    relief=np.clip(.5+noise+ridges,.06,.94)
    if texture:
        # Keep user-provided photographic image unmodified; geometry has its own raster.
        # Preserve JPEG bytes exactly; PNG/TIFF is converted to RGB JPG as a fallback.
        source=Path(texture)
        if source.suffix.lower() in ('.jpg','.jpeg'):
            img_bytes=source.read_bytes()
            with Image.open(source) as check: check.verify()
            img=None
        else:
            img=Image.open(source).convert('RGB')
            img_bytes=None
    else:
        sand=np.asarray(Image.open(Path(__file__).resolve().parent.parent/'data'/'photogrammetry'/'textures'/'seabed.jpg').convert('RGB').resize((side,side)),dtype=np.float32) if (out/'textures'/'seabed.jpg').exists() else None
        if sand is None:
            a=rng.normal(size=(side,side)); grains=np.clip(12*a+12*(relief-.5),-24,24)
            sand=np.stack((151+grains,146+grains*.92,119+grains*.75),axis=2).clip(0,255)
        img=Image.fromarray(np.uint8(sand),'RGB')
    if height_source:
        him=Image.open(height_source).convert('L').resize((side,side),Image.Resampling.LANCZOS)
        a=np.asarray(him,dtype=np.float32)/255.0
        # Deliberately smooth photographic / compressed input to avoid spikes.
        relief=np.clip(smooth(a,3.0)*.75+.125,.05,.95)
    # Shape stable level pads at survey sites. X spans 88 m. Image y-axis is
    # inverted relative to the world y-axis, matching the map preview.
    metres=88.0
    centres=[(-10.,-6.,3.0),(-10.,5.,3.0),(19.,6.,13.8),(-16.,-6.,1.0)]
    for x,y,rad in centres:
        px=(x/metres+.5)*(side-1);py=(.5-y/metres)*(side-1)
        d=np.sqrt((np.arange(side)[None,:]-px)**2+(np.arange(side)[:,None]-py)**2)*metres/(side-1)
        w=np.clip((rad+2.0-d)/2.0,0,1); w=w*w*(3-2*w)
        relief=relief*(1-w)+.5*w
    # Pixel 0 -> 0m relief, pixel 65535 -> amplitude metres relief.
    # Stonefish terrain world_transform z=10m, height=amplitude in scene.
    Image.fromarray(np.uint16(np.clip(relief,0,1)*65535),'I;16').save(out/'terrain_height.png')
    if texture and img_bytes is not None:
        (out/'textures'/'seabed.jpg').write_bytes(img_bytes)
    else:
        img.save(out/'textures'/'seabed.jpg',quality=95,optimize=True)
    report={'size_pixels':side,'horizontal_span_m':metres,'metres_per_pixel':metres/side,'max_height_m':amplitude,
            'mean_normalized_height':float(relief.mean()),'terrain_z_origin_m':10.0,
            'texture_input':str(texture) if texture else 'generated procedural sand',
            'height_source':str(height_source) if height_source else 'artificial multiscale relief'}
    (out/'terrain_manifest.json').write_text(__import__('json').dumps(report,indent=2)+'\n')
    return report

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--texture',help='RGB seabed texture image, ideally top-down and tileable')
    p.add_argument('--height-source',help='Optional grayscale map for terrain relief')
    p.add_argument('--size',type=int,default=512)
    p.add_argument('--output',type=Path,default=None,help='Output photogrammetry directory')
    a=p.parse_args();print(generate(a.texture,a.height_source,a.size,out=a.output))
if __name__=='__main__':main()
