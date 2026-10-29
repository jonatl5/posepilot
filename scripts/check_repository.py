"""Check publishable text for language and machine-local paths."""
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]
TEXT = {'.py', '.h', '.cpp', '.md', '.html', '.css', '.js', '.mjs', '.json',
        '.toml', '.txt', '.jsonl', '.csv', '.yaml', '.yml', '.svg', '.gitignore'}


def main():
    if (ROOT/'.git').exists():
        paths = subprocess.check_output(
            ['git', 'ls-files', '-z', '--cached', '--others', '--exclude-standard'],
            cwd=ROOT).decode().split('\0')
    else:
        excluded = {'.venv', 'node_modules', '__pycache__', 'runs', 'artifacts', 'build', 'dist'}
        paths = [p.relative_to(ROOT).as_posix() for p in ROOT.rglob('*')
                 if p.is_file() and not excluded.intersection(p.relative_to(ROOT).parts)]
    checked = 0
    issues = []
    for name in sorted(set(paths)-{''}):
        path = ROOT/name
        if not path.is_file() or (path.suffix not in TEXT and path.name not in ('LICENSE', '.gitignore')):
            continue
        value = path.read_text(encoding='utf-8')
        checked += 1
        if re.search(r'[\u3400-\u9fff\u3040-\u30ff\uac00-\ud7af]', value):
            issues.append(f'{name}: unexpected non-English text')
        if re.search(r'/(?:Users|home)/[A-Za-z0-9_-]+/', value):
            issues.append(f'{name}: machine-local absolute path')
    if issues:
        raise SystemExit('\n'.join(issues))
    print(f'Checked {checked} publishable text files, including code comments.')


if __name__ == '__main__':
    main()
