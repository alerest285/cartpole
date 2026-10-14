"""Exercise the actual browser console against the local simulation server."""
import json
from pathlib import Path
from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "docs/evidence"
OUT.mkdir(exist_ok=True)
errors = []
with sync_playwright() as p:
    browser = p.chromium.launch(executable_path="/Users/ale/Library/Caches/force-gello-review/chrome-headless-shell-mac-arm64/chrome-headless-shell", headless=True)
    page = browser.new_page(viewport={"width":1440,"height":950})
    page.on("pageerror",lambda e:errors.append(str(e)))
    response=page.goto('http://127.0.0.1:8000')
    assert response.status==200
    page.wait_for_function('S.status && S.status.backend.sim')
    # This script must never arm a hardware server.
    assert page.evaluate('S.status.backend.sim')
    page.locator('#stop').click()
    page.locator('#reset-hang').click()
    page.locator('#mode').select_option('energy')
    page.wait_for_function('S.status.mode === "energy"')
    page.locator('#arm').click()
    page.wait_for_function('S.status.armed && Math.abs(S.status.s[1]) < 0.09',timeout=15000)
    page.wait_for_timeout(3000)
    energy=page.evaluate('S.status')
    assert energy['armed'] and abs(energy['s'][1])<0.09, energy
    page.screenshot(path=str(OUT/'browser-energy.png'),full_page=True)
    page.locator('#stop').click()
    page.wait_for_function('!S.status.armed')
    page.locator('#reset-hang').click()
    page.locator('#mode').select_option('ppo')
    page.wait_for_function('S.status.mode === "ppo"')
    page.locator('#arm').click()
    page.wait_for_function('S.status.armed && Math.abs(S.status.s[1]) < 0.09',timeout=15000)
    page.wait_for_timeout(3000)
    ppo=page.evaluate('S.status')
    assert ppo['armed'] and abs(ppo['s'][1])<0.09, ppo
    page.screenshot(path=str(OUT/'browser-ppo.png'),full_page=True)
    page.locator('#stop').click()
    page.wait_for_function('!S.status.armed && S.status.v === 0')
    page.locator('[data-tab="runs"]').click()
    page.wait_for_function('S.runs.length >= 2',timeout=6000)
    assert page.locator('#runs-table tbody tr').count()>=2
    page.screenshot(path=str(OUT/'browser-runs.png'),full_page=True)
    page.locator('[data-tab="control"]').click()
    page.locator('#mode').select_option('energy')
    page.locator('#reset-hang').click()
    assert not errors,errors
    report={'energy_alpha_deg':energy['s'][1]*180/3.141592653589793,
            'ppo_alpha_deg':ppo['s'][1]*180/3.141592653589793,
            'stopped_at_zero_volts':True,'page_errors':errors,'simulation_only':True}
    (OUT/'browser-check.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    browser.close()
