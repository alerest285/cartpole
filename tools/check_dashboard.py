"""Dashboard browser acceptance, with isolated simulated runs and model data."""
import json
import os
import shutil
import signal
import socket
import subprocess
import tempfile
import time
from pathlib import Path

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parent.parent


def check():
    with tempfile.TemporaryDirectory(prefix="cartpole-dashboard-") as tmp:
        data=Path(tmp)/"data";data.mkdir()
        shutil.copytree(ROOT/"data/policies",data/"policies")
        shutil.copy2(ROOT/"data/bench.json",data/"bench.json")
        env=dict(os.environ,CARTPOLE_DATA=str(data),CARTPOLE_RUNS=str(Path(tmp)/"runs"),OPENBLAS_NUM_THREADS="1")
        with socket.socket() as sock:
            sock.bind(("127.0.0.1",0));port=sock.getsockname()[1]
        server=subprocess.Popen([str(ROOT/".venv/bin/python"),"-m","cartpole","serve","--demo","--no-browser","--port",str(port)],cwd=ROOT,env=env,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
        try:
            for _ in range(60):
                try:
                    with socket.create_connection(("127.0.0.1",port),timeout=.2): break
                except OSError: time.sleep(.1)
            with sync_playwright() as pw:
                options={"headless":True}
                if os.getenv("PLAYWRIGHT_CHROMIUM_EXECUTABLE"): options["executable_path"]=os.environ["PLAYWRIGHT_CHROMIUM_EXECUTABLE"]
                browser=pw.chromium.launch(**options)
                page=browser.new_page(viewport={"width":1440,"height":1050},color_scheme="light")
                errors=[];page.on("pageerror",lambda e:errors.append(str(e)))
                page.goto(f"http://127.0.0.1:{port}")
                page.wait_for_function('S.status?.armed && document.querySelectorAll(".course-card").length===6')
                assert page.locator("#dashboard").is_visible()
                assert page.locator("#scene").is_visible()
                assert page.locator("#analyze-friction").is_disabled()
                page.wait_for_function('Math.abs(S.status.s[1]) < .09 && S.status.ctrl.info.phase === "balancing"',timeout=15000)
                page.screenshot(path=str(ROOT/"docs/evidence/dashboard-desktop.png"),full_page=True)
                page.locator('[data-doc="tomorrow"]').click()
                page.wait_for_function('document.querySelector("#document-content").textContent.includes("friction")')
                page.locator("#close-document").click()
                for key in ("handout","model","bundle"):
                    response=page.request.get(f"http://127.0.0.1:{port}/materials/{key}");assert response.ok,key
                    assert response.body().startswith(b"PK" if key=="bundle" else b"%PDF")
                page.locator("#check-0").check()
                page.locator("#stop").click();page.wait_for_function("!S.status.armed")
                page.reload();page.wait_for_function("S.status !== null")
                assert not page.evaluate("S.status.armed")
                assert page.locator("#check-0").is_checked()
                page.locator("#friction-file").set_input_files(str(ROOT/"docs/evidence/lab-rehearsal/simulated-regression.csv"))
                page.locator("#friction-delay").fill("1")
                page.locator("#analyze-friction").click()
                page.wait_for_selector("#friction-result table")
                assert "SIM" in page.locator("#friction-result").inner_text()
                page.locator("#import-recording").click()
                page.wait_for_function('S.runs.some(r=>r.mode==="import" && r.sim)')
                imported=page.evaluate('S.runs.find(r=>r.mode==="import")')
                page.locator("#friction-run").select_option(imported["id"])
                assert page.locator("#upload-fields").is_hidden()
                page.locator("#analyze-friction").click();page.wait_for_function('document.querySelector("#friction-result").textContent.includes("'+imported["id"]+'")')
                assert not (data/"params_fitted.json").exists()
                page.locator('[data-job="modeling"]').click()
                page.wait_for_function('document.querySelector("#dashboard-joblog").textContent.includes("modeling: completed")',timeout=45000)
                assert not page.evaluate("S.status.armed")
                # Selection opens controls but never arms the controller.
                page.locator(".course-card").nth(3).locator("button").click()
                page.wait_for_function('S.status.mode==="statefb"')
                assert page.locator("#tab-control").is_visible()
                assert not page.evaluate("S.status.armed")
                page.locator("#show-dashboard").click()
                page.locator("#friction-run").select_option("")
                page.locator("#friction-file").set_input_files({"name":"bad.csv","mimeType":"text/csv","buffer":b"bad,columns\n1,2\n"})
                page.locator("#analyze-friction").click()
                page.wait_for_function('document.querySelector("#dashboard-notice").classList.contains("error")')
                assert "CSV needs" in page.locator("#friction-result").inner_text()
                for width in (390,800,1440):
                    page.set_viewport_size({"width":width,"height":950});page.wait_for_timeout(200)
                    assert page.evaluate("document.documentElement.scrollWidth <= innerWidth"),width
                    assert page.locator("#stop").is_visible()
                    if width==390: page.screenshot(path=str(ROOT/"docs/evidence/dashboard-mobile.png"),full_page=True)
                page.emulate_media(color_scheme="dark");page.screenshot(path=str(ROOT/"docs/evidence/dashboard-dark.png"),full_page=True)
                assert not errors,errors
                report={"overview_and_live_scene":True,"course_sections":6,"pdfs_and_bundle":True,
                        "stop_and_notes_survive_reload":True,"csv_friction_and_saved_run":True,
                        "imports_default_simulated":True,"model_not_auto_applied":True,
                        "offline_modeling_job":True,"controller_selection_stays_disarmed":True,
                        "bad_csv_rejected":True,"responsive_widths":[390,800,1440],"page_errors":errors}
                (ROOT/"docs/evidence/dashboard-browser.json").write_text(json.dumps(report,indent=2)+"\n")
                print(json.dumps(report,indent=2));browser.close()
        finally:
            server.send_signal(signal.SIGINT)
            try: server.wait(timeout=5)
            except subprocess.TimeoutExpired: server.kill();server.wait()
            err=server.stderr.read().decode();server.stderr.close()
            if err: print(err[-2500:])


if __name__=="__main__": check()
