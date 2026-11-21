"""Headless diagnostic, not a timing gate: hython tests/ko/adjust_perf.py."""
from pathlib import Path
from statistics import median
import sys
from time import perf_counter

import hou

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'python3.13libs'))
import ko_adjust as a


def run():
    hou.hda.installFile(str(ROOT/'otls/sop_ko.koadjust.1.0.hda'))
    obj=hou.node('/obj').createNode('geo',run_init_scripts=False)
    grid=obj.createNode('grid')
    grid.setParms({'orient':0,'rows':160,'cols':160,'sizex':2,'sizey':2})
    wave=obj.createNode('attribwrangle')
    wave.setInput(0,grid)
    wave.parm('snippet').set('@P.z = .03*sin(30*@P.x)*cos(25*@P.y);')
    node=a.make_adjust(wave)
    geo=wave.geometry()
    samples=[]
    for i in range(65):
        p,n,uv=hou.Vector3(),hou.Vector3(),hou.Vector3()
        hit=geo.intersect(hou.Vector3(-.81+i*.025,.05,1),hou.Vector3(0,0,-1),p,n,uv)
        samples.append((*p,*geo.prim(hit).normal()))
    stage=a.append(node,'smooth',center=samples[0][:3],normal=samples[0][3:],radius=.16)
    stage.node('deform').parm('strength').set(1.)
    bind=stage.node('bind')
    node.geometry()
    target=stage.node('smooth_target')
    cached_cooks=target.cookCount() if target is not None else None
    for count in (20,60):
        runs=[]
        for i in range(8):
            begin=perf_counter()
            bind.parm('samples').set(a.encode_samples(samples[1:count+(i%2)]))
            output=node.geometry()
            runs.append((perf_counter()-begin)*1000)
            assert output is not None and not a.status(stage),a.status(stage)
        print(f'{geo.pointCount()} points, {count}/{count+1} samples: {median(runs):.2f} ms median, {max(runs):.2f} ms max (Smooth recook)')
    if target is not None:
        assert target.cookCount()==cached_cooks, 'Smooth target was recooked on cursor changes.'


if __name__=='__main__':
    run()
