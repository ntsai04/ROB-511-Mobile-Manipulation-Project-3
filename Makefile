PYTHON ?= python3

.PHONY: build run demo viewer record portfolio test submission clean

build:
	$(PYTHON) -m compileall -q src tools tests

run:
	$(PYTHON) src/main.py

demo:
	$(PYTHON) src/main.py --demo

viewer:
	$(PYTHON) tools/viewer_server.py

record:
	$(PYTHON) tools/record_demo.py --duration 24 --output portfolio/demo_recording.json

portfolio:
	$(PYTHON) tools/package_portfolio.py

test:
	$(PYTHON) -m unittest discover -s tests -v

submission:
	$(PYTHON) tools/package_submission.py

clean:
	$(PYTHON) -c "import pathlib, shutil; [shutil.rmtree(p) for p in pathlib.Path('.').rglob('__pycache__')]"
	$(PYTHON) -c "from pathlib import Path; Path('submission.tar.gz').unlink(missing_ok=True)"
	$(PYTHON) -c "from pathlib import Path; Path('portfolio-site.zip').unlink(missing_ok=True)"
