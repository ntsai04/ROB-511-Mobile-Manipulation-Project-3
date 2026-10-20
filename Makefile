PYTHON ?= python3

.PHONY: build run test submission clean

build:
	$(PYTHON) -m compileall -q src tools tests

run:
	$(PYTHON) src/main.py

test:
	$(PYTHON) -m unittest discover -s tests -v

submission:
	$(PYTHON) tools/package_submission.py

clean:
	$(PYTHON) -c "import pathlib, shutil; [shutil.rmtree(p) for p in pathlib.Path('.').rglob('__pycache__')]"
	$(PYTHON) -c "from pathlib import Path; Path('submission.tar.gz').unlink(missing_ok=True)"
