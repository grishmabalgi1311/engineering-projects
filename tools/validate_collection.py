from pathlib import Path
import ast,hashlib,json
root=Path(__file__).resolve().parents[1]
count=0
for p in root.glob('projects/**/*.py'):
    ast.parse(p.read_text(encoding='utf-8-sig'),filename=str(p));count+=1
for record in json.loads((root/'FILE_INTEGRITY.json').read_text()):
    p=root/record['destination'].replace('\\','/')
    assert p.is_file(),f'Missing source artifact: {p}'
    assert hashlib.sha256(p.read_bytes()).hexdigest()==record['sha256'],f'Archived file changed: {p}'
comparison=json.loads((root/'projects/visual-navigation/evidence/midterm-comparison.json').read_text())
assert comparison['baseline']['ticks']-comparison['submitted']['ticks']==comparison['ticks_saved']==1033
assert abs(comparison['tick_reduction_percent']-42.14606283149735)<1e-6
assert len(list((root/'projects').glob('*/README.md')))==20
print(f'{count} Python source files parse; archived file hashes and navigation comparison verified. Hardware, native-toolchain and simulator execution are separate checks.')
