PYTHON ?= python3
VISUALIZER_PORT ?= 8003
RUNTIME_HOST ?= 127.0.0.1
RUNTIME_PORT ?= 9095

.PHONY: build run test visualizer viewer demo submission clean

build:
	@$(PYTHON) -m compileall -q src tools

run:
	@$(PYTHON) src/main.py

test:
	@$(PYTHON) tests/run_tests.py $(TESTS)

visualizer:
	@$(PYTHON) tools/visualizer.py --port $(VISUALIZER_PORT) --runtime-host $(RUNTIME_HOST) --runtime-port $(RUNTIME_PORT)

viewer: visualizer

demo:
	@$(PYTHON) src/main.py --demo

submission:
	@$(PYTHON) tools/package_submission.py

clean:
	@$(PYTHON) -c "import pathlib, shutil; [shutil.rmtree(p, ignore_errors=True) for p in pathlib.Path('.').rglob('__pycache__')]"
