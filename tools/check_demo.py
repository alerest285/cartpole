"""Check the actual first-launch demo, restart from Setup, and STOP semantics."""
import json
import signal
import socket
import subprocess
import time
from pathlib import Path
from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parent.parent
with socket.socket() as s:
    s.bind(('127.0.0.1',0)); port=s.getsockname()[1]
server=subprocess.Popen([str(ROOT/'.venv/bin/python'),'-m','cartpole','serve','--backend','sim',
                         '--demo','--no-browser','--port',str(port)],cwd=ROOT,
                        stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
try:
    with sync_playwright() as p:
        browser=p.chromium.launch(executable_path='/Users/ale/Library/Caches/force-gello-review/chrome-headless-shell-mac-arm64/chrome-headless-shell',headless=True)
        page=browser.new_page(viewport={'width':1440,'height':950})
        errors=[]; page.on('pageerror',lambda error:errors.append(str(error)))
        for attempt in range(50):
            try:
                with socket.create_connection(('127.0.0.1',port),timeout=0.2): break
            except OSError: time.sleep(0.1)
        page.goto(f'http://127.0.0.1:{port}')
        # No mode selection or Arm click: first browser must see the demo run.
        page.wait_for_function('S.status?.armed && S.status.mode === "energy"')
        page.wait_for_function('Math.abs(S.status.s[1]) < 0.09',timeout=15000)
        page.wait_for_timeout(2500)
        assert page.evaluate('S.status.armed && Math.abs(S.status.s[1]) < 0.09')
        page.screenshot(path=str(ROOT/'docs/evidence/demo-startup.png'),full_page=True)
        page.locator('[data-tab="setup"]').click()
        page.locator('#demo').click()
        page.wait_for_function('S.status.armed && Math.abs(S.status.s[1]) > 2')
        assert page.locator('#tab-control').is_visible()
        page.wait_for_function('Math.abs(S.status.s[1]) < 0.09',timeout=15000)
        page.locator('#stop').click()
        page.wait_for_function('!S.status.armed && S.status.v === 0')
        page.reload()
        page.wait_for_function('S.status !== null')
        assert page.evaluate('!S.status.armed'), 'Reload restarted a stopped demo'
        assert page.locator('#sim-hint').is_visible()
        assert page.locator('#demo').inner_text()=='Run demo'
        assert not errors,errors
        report={'first_launch_auto_swingup':True,'balanced_after_swingup':True,
                'restart_from_setup':True,'stop_survives_reload':True,'page_errors':errors}
        (ROOT/'docs/evidence/demo-check.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2)); browser.close()
finally:
    server.send_signal(signal.SIGINT)
    try:server.wait(timeout=5)
    except subprocess.TimeoutExpired:server.kill();server.wait()
