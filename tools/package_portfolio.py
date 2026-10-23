"""Create a static, portable website ZIP ready for a future GitHub Pages repo."""
from pathlib import Path
import zipfile


ROOT = Path(__file__).resolve().parents[1]


def main():
    output = ROOT / "portfolio-site.zip"
    missing = [str(path.relative_to(ROOT)) for path in
               (ROOT / "portfolio" / "demo.webm", ROOT / "portfolio" / "demo_recording.json")
               if not path.is_file()]
    if missing:
        raise SystemExit("Create the portfolio recording and video first; missing " + ", ".join(missing))
    redirect = ('<!doctype html><html lang="en"><meta charset="utf-8">'
                '<meta name="viewport" content="width=device-width,initial-scale=1">'
                '<meta http-equiv="refresh" content="0;url=portfolio/">'
                '<title>Luma robotics portfolio</title>'
                '<a href="portfolio/">Open the Luma robotics portfolio</a></html>')
    with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("index.html", redirect)
        archive.writestr(".nojekyll", "")
        for directory in ("portfolio", "viewer"):
            for path in sorted((ROOT / directory).rglob("*")):
                if not path.is_file() or "__pycache__" in path.parts or path.suffix == ".tmp":
                    continue
                name = path.relative_to(ROOT).as_posix()
                if name == "portfolio/index.html":
                    page = path.read_text(encoding="utf-8").replace(
                        'href="../viewer/"', 'href="../viewer/?replay=../portfolio/demo_recording.json"'
                    ).replace("Open live viewer", "Open recorded viewer")
                    archive.writestr(name, page)
                else:
                    archive.write(path, name)
    print(f"Wrote {output} ({output.stat().st_size:,} bytes). Extract it without changing folder layout.")


if __name__ == "__main__":
    main()
