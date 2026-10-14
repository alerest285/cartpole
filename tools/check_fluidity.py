"""Browser verification of motion, plots, resets, resize, STOP and lost telemetry.

Requires Playwright and Chromium. Optionally set PLAYWRIGHT_CHROMIUM_EXECUTABLE.
Uses an isolated simulator port; never connects to hardware.
"""
import json
import os
import signal
import socket
import statistics
import subprocess
import time
from pathlib import Path

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parent.parent
EVIDENCE = ROOT / 'docs/evidence'


def check():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    server = subprocess.Popen(
        [str(ROOT / '.venv/bin/python'), '-m', 'cartpole', 'serve', '--backend', 'sim',
         '--demo', '--no-browser', '--port', str(port)], cwd=ROOT,
        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
    )
    try:
        for _ in range(50):
            try:
                with socket.create_connection(('127.0.0.1', port), timeout=0.2):
                    break
            except OSError:
                time.sleep(0.1)
        with sync_playwright() as p:
            options = {'headless': True}
            if os.getenv('PLAYWRIGHT_CHROMIUM_EXECUTABLE'):
                options['executable_path'] = os.environ['PLAYWRIGHT_CHROMIUM_EXECUTABLE']
            browser = p.chromium.launch(**options)
            page = browser.new_page(viewport={'width': 1440, 'height': 950})
            errors = []
            page.on('pageerror', lambda error: errors.append(str(error)))
            page.goto(f'http://127.0.0.1:{port}')
            page.wait_for_function('S.status?.armed && S.status.mode === "energy"')
            page.locator('#show-console').click()
            page.evaluate('''() => {
              window.motionFrames = [];
              const original = frame;
              frame = function(now) {
                const start = performance.now(); original(now);
                if (visualPose && timeline.count) motionFrames.push({
                  wall: now, ...visualPose, cost: performance.now() - start,
                  lag: (timeline.row(timeline.count - 1)[0] - visualPose.t) * 1000
                });
              };
            }''')
            page.locator('[data-tab="setup"]').click()
            page.locator('#demo').click()
            assert page.locator('#tab-control').is_visible()
            page.wait_for_timeout(6500)
            assert page.evaluate('S.status.armed && Math.abs(S.status.s[1]) < 0.09')
            frames = page.evaluate('motionFrames')
            swing = [r for r in frames if r['wall'] - frames[0]['wall'] < 3000]
            poses = sum((a['x'], a['a']) != (b['x'], b['a']) for a, b in zip(swing, swing[1:]))
            plot_steps = sum(a['t'] != b['t'] for a, b in zip(swing, swing[1:]))
            costs = sorted(r['cost'] for r in frames)
            report = {
                'frames': len(frames), 'swing_seconds': 3,
                'pose_updates_per_second': poses / 3,
                'plot_updates_per_second': plot_steps / 3,
                'frame_p95_ms': costs[int(0.95 * len(costs))],
                'buffer_lag_median_ms': statistics.median(r['lag'] for r in frames),
                'first_launch_auto_demo': True, 'restart_from_setup': True,
                'balanced_after_swingup': True,
            }
            assert poses / 3 > 50, report
            assert plot_steps / 3 > 50, report
            assert report['frame_p95_ms'] < 8, report
            page.screenshot(path=str(EVIDENCE / 'fluidity-live.png'), full_page=True)

            # Reset must snap, never rotate smoothly through intermediate angles.
            page.evaluate('motionFrames = []')
            page.locator('#reset-hang').click()
            page.wait_for_function('!S.status.armed && Math.abs(visualPose.a) > 3.1')
            reset = page.evaluate('motionFrames')
            assert all(abs(r['a']) < 0.1 or abs(r['a']) > 3.1 for r in reset)
            report['reset_without_fictitious_swing'] = True

            # Stop persists across reload, even on a server launched with --demo.
            page.locator('#demo').click()
            page.wait_for_function('S.status.armed')
            page.locator('#stop').click()
            page.wait_for_function('!S.status.armed && S.status.v === 0')
            page.reload()
            page.wait_for_function('S.status !== null && visualPose !== null')
            assert page.evaluate('!S.status.armed')
            report['stop_survives_reload'] = True

            page.locator('#show-console').click()
            page.locator('[data-tab="course"]').click()
            page.wait_for_function('document.querySelectorAll(".course-step").length === 6')
            pdf = page.request.get(f'http://127.0.0.1:{port}/course/handout.pdf')
            assert pdf.ok and pdf.headers['content-type'] == 'application/pdf'
            assert pdf.body().startswith(b'%PDF')
            report['course_guide_and_original_pdf'] = True

            page.locator('[data-tab="setup"]').click()
            page.locator('#estimator').select_option('derivative')
            page.wait_for_function('S.status.estimator === "derivative"')
            page.locator('#derivative-w').fill('80')
            page.locator('#derivative-w').press('Tab')
            page.wait_for_function('S.status.estimator_w === 80')
            page.locator('#estimator').select_option('kalman')
            page.wait_for_function('S.status.estimator === "kalman"')
            report['derivative_cutoff_tuning'] = True

            page.locator('[data-tab="control"]').click()
            page.locator('#reset-hang').click()
            page.wait_for_function('Math.abs(S.status.s[1]) > 3.1')
            page.locator('#mode').select_option('manual_swing')
            page.wait_for_function('S.status.mode === "manual_swing"')
            page.locator('#arm').click()
            page.wait_for_function('S.status.armed')
            pad = page.locator('#manual-pad').bounding_box()
            page.mouse.move(pad['x'] + 40, pad['y'] + 20)
            page.mouse.down()
            page.mouse.move(pad['x'] + 120, pad['y'] + 20, steps=5)
            assert float(page.locator('#manual').input_value()) > 0
            page.wait_for_timeout(500)
            assert page.evaluate('S.status.ref.manual === 0')
            page.mouse.up()
            page.locator('#stop').click()
            page.wait_for_function('!S.status.armed && S.status.v === 0')
            report['manual_pad_stops_when_motion_stops'] = True

            # Canvas sizing/theme caches must update after resizing or opening a tab.
            for width in (800, 390, 1440):
                page.set_viewport_size({'width': width, 'height': 950})
                page.wait_for_timeout(150)
                sizes = page.evaluate('''() => ({
                  viewport: innerWidth, content: document.documentElement.scrollWidth,
                  css: document.querySelector('#scene').clientWidth,
                  pixels: document.querySelector('#scene').width, dpr: devicePixelRatio
                })''')
                assert sizes['content'] <= sizes['viewport'], sizes
                assert abs(sizes['css'] * sizes['dpr'] - sizes['pixels']) <= 2, sizes
            page.locator('[data-tab="learn"]').click()
            page.wait_for_timeout(150)
            assert page.locator('#ch-learn').evaluate('(c) => c.width > 0 && c.height > 0')
            page.emulate_media(color_scheme='dark')
            page.wait_for_timeout(150)
            assert page.evaluate('css("--bg")') == '#0e1013'
            page.emulate_media(color_scheme='light')
            page.wait_for_timeout(150)
            assert page.evaluate('css("--bg")') == '#f4f3ef'
            report['resize_mobile_tabs_and_theme'] = True

            # Disable reconnect for this page only, then interrupt its live stream.
            page.evaluate('connect = () => {}; ws.close()')
            page.wait_for_timeout(300)
            before = page.evaluate('({...visualPose})')
            page.wait_for_timeout(400)
            after = page.evaluate('({...visualPose})')
            assert before == after, (before, after)
            assert 'disconnected' in page.locator('#backend').inner_text()
            report['lost_telemetry_freezes_display'] = True
            assert not errors, errors
            report['page_errors'] = errors
            (EVIDENCE / 'fluidity-after.json').write_text(json.dumps(report, indent=2) + '\n')
            print(json.dumps(report, indent=2))
            browser.close()
    finally:
        server.send_signal(signal.SIGINT)
        try:
            server.wait(timeout=5)
        except subprocess.TimeoutExpired:
            server.kill()
            server.wait()


if __name__ == '__main__':
    check()
