# Portable robotics portfolio

The page, video, and recorded interactive replay work on an ordinary static web host.
Nothing needs to connect to the robot runtime when viewing the recorded replay.

Preview from the project root with `python tools/viewer_server.py`, then open
`http://127.0.0.1:8080/portfolio/`. The live viewer at `/viewer/` additionally connects to
the running robot on TCP port 9095.

To refresh the media:

1. Start the demo with `python src/main.py --demo`.
2. In another terminal, run `python tools/record_demo.py --duration 24`.
3. Open `http://127.0.0.1:8080/viewer/?replay=../portfolio/demo_recording.json`.
4. Select **Export video** and keep the tab open for the duration of the recording. The
   local server saves `portfolio/demo.webm`. Select **Save still frame** for `poster.png`.

Run `python tools/package_portfolio.py` to create `portfolio-site.zip`. The ZIP contains
only the static page, its media, and the viewer, with an `index.html` that opens the portfolio.
Extract it into a future website repository without changing the folder layout.

For a future GitHub Pages site, push those extracted files to your repository, then select
that branch and its root folder in the repository's Pages settings. No build command, Python
server, external library, or credentials are included or needed. The packaged navigation opens
the recorded viewer. This task creates the portable files locally; it does not publish a site.

The renderer supports box, cylinder, and sphere visuals. It reads the first visual's local
origin and material and composes that origin with each recorded column-major world matrix.
External mesh files are not fetched; a robot containing only mesh visuals uses a frame skeleton.
