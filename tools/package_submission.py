"""Package this flattened Python project with its Makefile at the archive root."""

from pathlib import Path
import tarfile


ROOT = Path(__file__).resolve().parents[1]
INCLUDE = ("Makefile", "README.md", "make_submission.sh", "src", "tools", "tests",
           "spec", "docs", "robots", "examples", "viewer", "portfolio")


def main() -> None:
    output = ROOT / "submission.tar.gz"

    def source_only(member: tarfile.TarInfo) -> tarfile.TarInfo | None:
        parts = Path(member.name).parts
        if "__pycache__" in parts or member.name.endswith((".pyc", ".pyo", ".tmp")):
            return None
        return member

    with tarfile.open(output, "w:gz") as archive:
        for name in INCLUDE:
            archive.add(ROOT / name, arcname=name, filter=source_only)
    with tarfile.open(output) as archive:
        assert "Makefile" in archive.getnames()
        assert all(not Path(name).is_absolute() and ".." not in Path(name).parts
                   for name in archive.getnames())
    print(f"Wrote {output} ({output.stat().st_size:,} bytes)")


if __name__ == "__main__":
    main()
