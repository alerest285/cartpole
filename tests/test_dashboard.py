"""Exercise the actual HTTP recording workflow and offline job safety gates."""
import json
import tempfile
import threading
import unittest
from dataclasses import replace
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from cartpole import dashboard, import_run, server
from cartpole.backends.sim import SimBackend
from cartpole.params import Plant, ROOT
from cartpole.runtime import Runtime


class Dashboard(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.folder = Path(self.tmp.name)
        self.patches = [patch.object(module, key, self.folder / key.lower())
                        for module, key in ((dashboard,"DATA"),(dashboard,"RUNS"),
                                            (import_run,"RUNS"),(server,"RUNS"))]
        for p in self.patches: p.start()
        plant = replace(Plant(), m_c=.60, B_f=3.3, k_amp=.93, B_p=.004, F_coul=.15, delay_steps=1)
        self.rt = Runtime(SimBackend(plant), plant, log=lambda _: None)
        self.app = server.App(self.rt)
        self.http = ThreadingHTTPServer(("127.0.0.1",0),server.make_handler(self.app))
        self.thread = threading.Thread(target=self.http.serve_forever,daemon=True);self.thread.start()
        self.base = "http://127.0.0.1:" + str(self.http.server_port)

    def tearDown(self):
        self.http.shutdown();self.http.server_close();self.thread.join()
        for p in reversed(self.patches): p.stop()
        self.tmp.cleanup()

    def request(self, path, payload=None, **headers):
        req = Request(self.base+path, data=None if payload is None else json.dumps(payload).encode(),
                      headers={"Content-Type":"application/json", **headers})
        try:
            with urlopen(req, timeout=10) as response: return response.status,response.read()
        except HTTPError as error:
            with error: return error.code,error.read()

    def test_csv_import_analysis_and_provenance(self):
        csv = (ROOT/"docs/evidence/lab-rehearsal/simulated-regression.csv").read_text()
        payload = {"csv":csv,"filename":"matlab.csv","delay_steps":1}
        code, body = self.request("/api/import",payload);self.assertEqual(code,200,body)
        imported = json.loads(body);self.assertTrue(imported["sim"])
        self.assertEqual(imported["uploaded_name"],"matlab.csv")
        code, body = self.request("/api/friction",{"run_id":imported["id"],"delay_steps":1})
        self.assertEqual(code,200,body); result=json.loads(body)
        self.assertTrue(result["accepted"]);self.assertTrue(result["source"]["sim"])
        self.assertAlmostEqual(result["params"]["B_f"],3.3,delta=.3)
        self.assertAlmostEqual(self.rt.plant.B_f,3.3)  # analysis never installs
        self.assertTrue((dashboard.DATA/"friction_latest.json").is_file())
        code, body = self.request("/api/import",{**payload,"hardware":True})
        self.assertEqual(code,200);self.assertFalse(json.loads(body)["sim"])

    def test_stopped_only_analysis_and_rejected_uploads(self):
        self.rt.armed=True
        self.assertEqual(self.request("/api/friction",{"csv":"bad"})[0],409)
        self.rt.armed=False
        for payload in ({"csv":"wrong,columns\n1,2\n"},{"run_id":"../data/private"},{"delay_steps":7,"csv":"x"}):
            self.assertEqual(self.request("/api/friction",payload)[0],400)
        self.assertEqual(self.request("/api/import",{"csv":"bad"})[0],400)
        self.assertFalse(list((dashboard.DATA/"imports").glob("*.csv")))
        self.assertEqual(self.request("/api/import",{"csv":"x"},Origin="https://unrelated.example")[0],403)

    def test_only_allowlisted_materials_and_reports(self):
        for path in ("/materials/handout","/materials/model","/materials/tomorrow","/api/dashboard"):
            self.assertEqual(self.request(path)[0],200,path)
        for path in ("/materials/../../.git/config","/api/evidence/../../params","/api/runs/../../data/bench.json"):
            self.assertEqual(self.request(path)[0],404,path)

    def test_job_allowlist_and_arm_gate(self):
        class Sink:
            def __init__(self): self.messages=[]
            def send(self,value): self.messages.append(value)
        sink=Sink();self.app.handle(sink,{"type":"lab_job","task":"arbitrary_script"})
        self.assertIn("Unknown lab task",sink.messages[-1]["msg"])
        self.rt.armed=True;self.app.handle(sink,{"type":"lab_job","task":"tune"})
        self.assertIn("Stop",sink.messages[-1]["msg"])
        self.rt.armed=False
        with patch.object(self.app.jobs,"running",return_value=True):
            self.app.handle(sink,{"type":"arm"});self.assertFalse(self.rt.armed)
            self.app.handle(sink,{"type":"demo"});self.assertFalse(self.rt.armed)
        with patch.object(self.app.jobs,"start",return_value=None) as start:
            self.app.handle(sink,{"type":"lab_job","task":"export_matlab"})
            self.assertEqual(start.call_args.kwargs["command"][-1],"tools/export_matlab.py")
