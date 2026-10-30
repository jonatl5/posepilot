"""Build a reproducible source download from an explicit repository allowlist."""
from pathlib import Path
import hashlib
import json
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    names = {'.gitignore', 'LICENSE', 'README.md', 'TASKS.md', 'PRD.md', 'pyproject.toml',
             'requirements-dev.txt', 'requirements-rl.txt', 'requirements-yolo.txt',
             'data/clips.example.csv', 'scripts/check_repository.py', 'scripts/package_source.py'}
    for directory, extensions in {
        'src': {'.py', '.html'}, 'tests': {'.py'}, 'configs': {'.json'}, '.github': {'.yml'},
        'firmware': {'.py', '.h', '.md'}, 'research': {'.md', '.json', '.txt'},
        'docs/evidence': {'.json', '.jsonl', '.csv', '.md'},
    }.items():
        names.update(p.relative_to(ROOT).as_posix() for p in (ROOT/directory).rglob('*')
                     if p.is_file() and p.suffix in extensions and '__pycache__' not in p.parts)
    # Include the complete static site and its maintenance scripts for README links.
    for directory in ('docs', 'scripts'):
        names.update(p.relative_to(ROOT).as_posix() for p in (ROOT/directory).rglob('*')
                     if p.is_file() and p.suffix in {'.html', '.css', '.js', '.mjs', '.png', '.svg'})
    names.update({'package.json', 'docs/.nojekyll', 'docs/PRD.md', 'docs/data/prd.json'})
    output = ROOT/'docs/assets/objectpilot-starter.zip'
    output.parent.mkdir(parents=True, exist_ok=True)
    manifest = {}
    with zipfile.ZipFile(output, 'w', zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
        for name in sorted(names):
            value = (ROOT/name).read_bytes()
            manifest[name] = hashlib.sha256(value).hexdigest()
            info = zipfile.ZipInfo('objectpilot/'+name, date_time=(2026, 10, 29, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o644 << 16
            archive.writestr(info, value)
        info = zipfile.ZipInfo('objectpilot/SOURCE_MANIFEST.json', date_time=(2026, 10, 29, 0, 0, 0))
        info.compress_type = zipfile.ZIP_DEFLATED
        archive.writestr(info, json.dumps(manifest, indent=2)+'\n')
    print(json.dumps({'archive': output.relative_to(ROOT).as_posix(), 'files': len(names),
                      'sha256': hashlib.sha256(output.read_bytes()).hexdigest()}, indent=2))


if __name__ == '__main__':
    main()
