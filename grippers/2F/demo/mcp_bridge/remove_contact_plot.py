"""
Tear down the live gripper contact-force plot created by make_contact_plot.py.

Connects to the Isaac MCP extension socket (127.0.0.1:8766) and destroys the
plot window plus its per-frame update subscription. Safe to run if no plot is
active. See make_contact_plot.py for the companion creator.

    python3 remove_contact_plot.py
"""
import json, socket
def send(cmd, t=70):
    s=socket.socket(); s.settimeout(t); s.connect(("127.0.0.1",8766))
    s.sendall(json.dumps(cmd).encode()); buf=b""
    while True:
        c=s.recv(65536)
        if not c: break
        buf+=c
        try: return json.loads(buf.decode())
        except: continue
    return {}
code=r'''
import traceback
try:
    import carb
    prev=getattr(carb,"_contactplot",None)
    if prev is None:
        print("OK: no contact plot was active (nothing to remove)")
    else:
        try: prev["sub"]=None
        except Exception: pass
        try: prev["win"].destroy()
        except Exception: pass
        try: carb._contactplot=None
        except Exception: pass
        print("OK: contact-force plot removed (window destroyed, per-frame subscription cancelled)")
except Exception:
    print("ERR:\n"+traceback.format_exc())
'''
r=send({"type":"simulation.execute_script","params":{"code":code}})
out=r.get("result",{})
print("\n".join(l for l in out.get("stdout","").splitlines() if not l.startswith("[mcp]")))
if out.get("stderr"): print("STDERR:", out["stderr"][-800:])
